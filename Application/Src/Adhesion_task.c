/**
  ******************************************************************************
  * @file       Adhesion_task.c
  * @brief      ����ϵͳ�������񣬻��ڷ������ʵ�ֿɿ���������
  * @note       ʹ��FreeRTOSʵ�֣�����bsp_fan����
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. ������
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "Adhesion_task.h"
#include "remote_receive.h"
#include "detect_task.h"
#include "cmsis_os.h"

/* ������ */
osThreadId adhesion_task_handle;

/* ��̬ȫ�ֱ��� */
static adhesion_info_t adhesion_info = {0};
static uint32_t last_control_time = 0;
static uint8_t is_initialized = 0;

/* ��̬�������� */
static void adhesion_control_update(void);
static void adhesion_mode_switch(void);
static float adhesion_rc_to_power(int16_t rc_value);
static void adhesion_safety_check(void);

/**
  * @brief          ����ϵͳ����ͨ��ң�������Ʒ��ʵ����������
  * @param[in]      argument: �������ָ��
  * @retval         void
  */
void Adhesion_task(void const * argument)
{
    /* �ȴ�����ģ���ʼ����� */
    vTaskDelay(ADHESION_TASK_INIT_TIME);
    
    /* ��ʼ����� */
    if (!is_initialized)
    {
        adhesion_info.state = ADHESION_STATE_ERROR;
        adhesion_info.error_code = 1; // δ��ʼ������
        while (1)
        {
            vTaskDelay(1000);
        }
    }
    
    /* ��ȡң��������ָ�� */
    adhesion_info.rc_data = get_remote_ch_point();
    
    /* ������ѭ�� */
    uint32_t wake_time = osKernelSysTick();
    
    while (1)
    {
        /* ���ң����������Ч�� */
        if (toe_is_error(DBUS_TOE))
        {
            /* ң������������ȫֹͣ */
            adhesion_emergency_stop();
        }
        else
        {
            /* ����ģʽ�л� */
            adhesion_mode_switch();
            
            /* ���¿����߼� */
            adhesion_control_update();
            
            /* ��ȫ��� */
            adhesion_safety_check();
        }
        
        /* ����ϵͳ����ʱ�� */
        if (adhesion_info.state == ADHESION_STATE_RUNNING)
        {
            adhesion_info.runtime += ADHESION_TASK_PERIOD;
        }
        
        /* ����ѭ����ʱ */
        wake_time += ADHESION_TASK_PERIOD;
        osDelayUntil(&wake_time);
    }
}

/**
  * @brief          ����ϵͳ��ʼ��
  * @param[in]      fan_tim: ���PWM��ʱ�����
  * @param[in]      fan_channel: ���PWMͨ��
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t adhesion_system_init(TIM_HandleTypeDef* fan_tim, uint32_t fan_channel)
{
    /* ��ʼ��������� */
    if (bsp_fan_init(fan_tim, fan_channel) != 0)
    {
        return 1;
    }
    
    /* ��ʼ������ϵͳ״̬ */
    adhesion_info.mode = ADHESION_MODE_OFF;
    adhesion_info.state = ADHESION_STATE_OFF;
    adhesion_info.power = 0.0f;
    adhesion_info.target_power = 0.0f;
    adhesion_info.runtime = 0;
    adhesion_info.fan_index = FAN_MAIN_INDEX;
    adhesion_info.error_code = 0;
    
    /* �������� */
    osThreadDef(adhesionTask, Adhesion_task, ADHESION_TASK_PRIO, 0, ADHESION_STK_SIZE);
    adhesion_task_handle = osThreadCreate(osThread(adhesionTask), NULL);
    
    if (adhesion_task_handle == NULL)
    {
        return 2;
    }
    
    /* ��ǳ�ʼ����� */
    is_initialized = 1;
    
    return 0;
}

/**
  * @brief          ��������ϵͳģʽ
  * @param[in]      mode: ����ģʽ
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t adhesion_set_mode(adhesion_mode_e mode)
{
    if (!is_initialized)
    {
        return 1;
    }
    
    /* ģʽ�л��߼� */
    if (mode != adhesion_info.mode)
    {
        switch (mode)
        {
            case ADHESION_MODE_OFF:
                /* �ر�ģʽ - ֹͣ��� */
                bsp_fan_set_speed(adhesion_info.fan_index, 0);
                adhesion_info.state = ADHESION_STATE_STOPPING;
                adhesion_info.power = 0.0f;
                adhesion_info.target_power = 0.0f;
                break;
                
            case ADHESION_MODE_MANUAL:
                /* �ֶ�ģʽ - ��ʼ����Ϊ0 */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 0.0f;
                break;
                
            case ADHESION_MODE_AUTO:
                /* �Զ�ģʽ - Ĭ��50%���� */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 50.0f;
                break;
                
            case ADHESION_MODE_SMART:
                /* ����ģʽ - ����Ӧ���� */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 30.0f;
                break;
                
            default:
                return 2;
        }
        
        adhesion_info.mode = mode;
    }
    
    return 0;
}

/**
  * @brief          ������������
  * @param[in]      power: ���ʰٷֱ�(0-100)
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t adhesion_set_power(float power)
{
    if (!is_initialized)
    {
        return 1;
    }
    
    /* ���ƹ��ʷ�Χ */
    if (power > 100.0f)
    {
        power = 100.0f;
    }
    else if (power < 0.0f)
    {
        power = 0.0f;
    }
    
    /* ����Ŀ�깦�� */
    adhesion_info.target_power = power;
    
    return 0;
}

/**
  * @brief          ��ȡ����ϵͳ��Ϣ
  * @param[out]     info: ����ϵͳ��Ϣ�ṹָ��
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t adhesion_get_info(adhesion_info_t* info)
{
    if (!is_initialized || info == NULL)
    {
        return 1;
    }
    
    /* ������Ϣ */
    *info = adhesion_info;
    
    return 0;
}

/**
  * @brief          ����ֹͣ����ϵͳ
  * @param[in]      none
  * @retval         none
  */
void adhesion_emergency_stop(void)
{
    /* ֱ�ӹرշ�� */
    bsp_fan_set_speed(adhesion_info.fan_index, 0);
    
    /* ����״̬ */
    adhesion_info.mode = ADHESION_MODE_OFF;
    adhesion_info.state = ADHESION_STATE_OFF;
    adhesion_info.power = 0.0f;
    adhesion_info.target_power = 0.0f;
}

/**
  * @brief          �������Ƹ��º���
  * @param[in]      none
  * @retval         none
  */
static void adhesion_control_update(void)
{
    uint32_t current_time = osKernelSysTick();
    
    /* ���Ƹ��¼������ */
    if (current_time - last_control_time < ADHESION_CONTROL_INTERVAL)
    {
        return;
    }
    last_control_time = current_time;
    
    /* ���ڵ�ǰģʽִ�п����߼� */
    switch (adhesion_info.mode)
    {
        case ADHESION_MODE_OFF:
            /* ȷ�����ֹͣ */
            if (adhesion_info.power > 0.0f)
            {
                bsp_fan_set_speed(adhesion_info.fan_index, 0);
                adhesion_info.power = 0.0f;
            }
            
            if (adhesion_info.state == ADHESION_STATE_STOPPING)
            {
                adhesion_info.state = ADHESION_STATE_OFF;
            }
            break;
            
        case ADHESION_MODE_MANUAL:
            /* �ֶ�����ģʽ - ����ң�������� */
            if (adhesion_info.rc_data != NULL)
            {
                float rc_power = adhesion_rc_to_power(adhesion_info.rc_data[ADHESION_POWER_CHANNEL]);
                adhesion_info.target_power = rc_power;
            }
            
            /* ���·���ٶ� */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* ��ȡ��ǰʵ�ʹ��� */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* ����״̬ */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
            
        case ADHESION_MODE_AUTO:
            /* �Զ�����ģʽ - ʹ��Ԥ�蹦�� */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* ��ȡ��ǰʵ�ʹ��� */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* ����״̬ */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
            
        case ADHESION_MODE_SMART:
            /* 
             * ���ܿ���ģʽ - ���Ը��ݴ�������������Ӧ��������
             * �˴���Ϊ�Զ����ƣ�ʵ��Ӧ���п�����Ӵ����������߼�
             */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* ��ȡ��ǰʵ�ʹ��� */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* ����״̬ */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
    }
    
    /* �����豸״̬ */
    detect_hook(ADHESION_TOE);
}

/**
  * @brief          ����ģʽ�л�����
  * @param[in]      none
  * @retval         none
  */
static void adhesion_mode_switch(void)
{
    /* ���ң����������Ч�� */
    if (adhesion_info.rc_data == NULL)
    {
        return;
    }
    
    /* ����ң��������λ���л�ģʽ */
    int16_t mode_channel = adhesion_info.rc_data[ADHESION_MODE_CHANNEL];
    
    if (switch_is_down(mode_channel))
    {
        /* ��������λ�� - �ر����� */
        adhesion_set_mode(ADHESION_MODE_OFF);
    }
    else if (switch_is_mid(mode_channel))
    {
        /* �������м�λ�� - �Զ�ģʽ */
        adhesion_set_mode(ADHESION_MODE_AUTO);
    }
    else if (switch_is_up(mode_channel))
    {
        /* ��������λ�� - �ֶ�ģʽ */
        adhesion_set_mode(ADHESION_MODE_MANUAL);
    }
}

/**
  * @brief          ��ң����ֵת��Ϊ���ʰٷֱ�
  * @param[in]      rc_value: ң����ͨ��ֵ(-660��660)
  * @retval         ���ʰٷֱ�(0-100)
  */
static float adhesion_rc_to_power(int16_t rc_value)
{
    /* ͨ��ң��������Ϊ��ֵ������Ϊ��ֵ */
    if (rc_value > -5 && rc_value < 5)
    {
        /* ң�������м�λ�ã�Ӧ������ */
        return 0.0f;
    }
    else if (rc_value <= 0)
    {
        /* ң����������ӳ��Ϊ0-100%���� */
        return (float)(-rc_value) / 660.0f * 100.0f;
    }
    else
    {
        /* ң�������ƣ�����Ϊ0���� */
        return 0.0f;
    }
}

/**
  * @brief          ����ϵͳ��ȫ���
  * @param[in]      none
  * @retval         none
  */
static void adhesion_safety_check(void)
{
    /* 
     * �˺���������Ӹ��ְ�ȫ��飬����:
     * 1. ����¶ȼ��
     * 2. �������
     * 3. �������
     * 4. ��ѹ���
     * 
     * �˴�Ϊ�򻯰汾��ʵ��Ӧ����Ӧ��չ�˹���
     */
     
    /* ʾ��: ����ʱ�����ʱ���͹��� */
    if (adhesion_info.runtime > 3600000) // 1Сʱ
    {
        if (adhesion_info.target_power > 80.0f)
        {
            adhesion_info.target_power = 80.0f;
        }
    }
}