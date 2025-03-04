
  /**
  ******************************************************************************
  * @file       car_task.c
  * @brief      С���������񣬻���ң����ʵ�ֲ���С������
  * @note       ʹ��FreeRTOSʵ�֣�����bsp_car����
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

#include "car_task.h"
#include "remote_receive.h"
#include "detect_task.h"
#include "cmsis_os.h"
#include <math.h>

/* ������ */
osThreadId car_task_handle;

/* ȫ�ֱ��� */
static car_info_t car_info = {0};
static const int16_t* rc_data = NULL;
static uint8_t is_initialized = 0;
static uint32_t last_rc_time = 0;
static uint32_t last_timeout_check = 0;

/* ��̬�������� */
static void car_mode_switch(void);
static void car_rc_control_update(void);
static void car_status_update(void);
static void car_safety_check(void);
static int16_t apply_deadband(int16_t value, int16_t deadband);
static float map_remote_to_speed(int16_t rc_value, float scale);

/**
  * @brief          С���������񣬴���ң�������벢����С���˶�
  * @param[in]      argument: �������
  * @retval         void
  */
void car_task(void const * argument)
{
    /* �ȴ�����ģ���ʼ����� */
    vTaskDelay(CAR_TASK_INIT_TIME);

    /* ��ʼ����� */
    if (!is_initialized)
    {
        car_info.state = CAR_STATE_ERROR;
        car_info.error_code = 1; // ��ʼ��ʧ�ܴ���
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    /* ��ȡң��������ָ�� */
    rc_data = get_remote_ch_point();
    if (rc_data == NULL)
    {
        car_info.state = CAR_STATE_ERROR;
        car_info.error_code = 2; // ң����ָ���ȡʧ��
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    /* ������ѭ�� */
    uint32_t wake_time = osKernelSysTick();

    while (1)
    {
        /* ��ȫ��� */
        car_safety_check();

        if (car_info.rc_connected)
        {
            /* ����ģʽ�л� */
            car_mode_switch();

            /* ����ң�������Ƹ��� */
            car_rc_control_update();
        }

        /* ����С��״̬ */
        car_status_update();

        /* ��������ʱ�� */
        car_info.runtime += CAR_TASK_PERIOD;

        /* ����������ʱ */
        wake_time += CAR_TASK_PERIOD;
        osDelayUntil(&wake_time);
    }
}

/**
  * @brief          С������ϵͳ��ʼ��
  * @param[in]      left_tim: ������ʱ�����
  * @param[in]      left_channel: ����PWMͨ��
  * @param[in]      right_tim: �ҵ����ʱ�����
  * @param[in]      right_channel: �ҵ��PWMͨ��
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t car_system_init(TIM_HandleTypeDef* left_tim, uint32_t left_channel,
                        TIM_HandleTypeDef* right_tim, uint32_t right_channel)
{
    /* ��ʼ���ײ����� */
    if (bsp_car_init(left_tim, left_channel, right_tim, right_channel) != 0)
    {
        return 1;
    }

    /* ��ʼ��С��״̬ */
    car_info.mode = CAR_MODE_STOP;
    car_info.state = CAR_STATE_STOP;
    car_info.forward_speed = 0.0f;
    car_info.turn_rate = 0.0f;
    car_info.left_speed = 0.0f;
    car_info.right_speed = 0.0f;
    car_info.runtime = 0;
    car_info.error_code = 0;
    car_info.rc_connected = 0;

    /* ���õ������ */
    bsp_car_set_motor_direction(CAR_LEFT_MOTOR_INDEX, MOTOR_DIR_NORMAL);
    bsp_car_set_motor_direction(CAR_RIGHT_MOTOR_INDEX, MOTOR_DIR_NORMAL);

    /* �������� */
    osThreadDef(carTask, car_task, CAR_TASK_PRIO, 0, CAR_STK_SIZE);
    car_task_handle = osThreadCreate(osThread(carTask), NULL);

    if (car_task_handle == NULL)
    {
        return 2;
    }

    /* ��ǳ�ʼ����� */
    is_initialized = 1;

    return 0;
}

/**
  * @brief          ����С������ģʽ
  * @param[in]      mode: ����ģʽ
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t car_set_mode(car_mode_e mode)
{
    if (!is_initialized)
    {
        return 1;
    }

    /* ģʽ�л��߼� */
    if (mode != car_info.mode)
    {
        switch (mode)
        {
            case CAR_MODE_STOP:
                /* ֹͣС�� */
                bsp_car_stop();
                car_info.forward_speed = 0.0f;
                car_info.turn_rate = 0.0f;
                break;

            case CAR_MODE_NORMAL:
                /* ��ͨģʽ - ��׼��Ӧ�ٶ� */
                break;

            case CAR_MODE_SPORT:
                /* �˶�ģʽ - �������� */
                break;

            case CAR_MODE_ECO:
                /* ����ģʽ - ����ʡ�� */
                break;

            case CAR_MODE_AUTO:
                /* �Զ�ģʽ - �������� */
                break;

            case CAR_MODE_EMERGENCY:
                /* ����ģʽ - ����ͣ�� */
                car_emergency_stop();
                return 0;

            default:
                return 2;
        }

        car_info.mode = mode;
    }

    return 0;
}

/**
  * @brief          �ֶ�����С���˶�
  * @param[in]      forward_speed: ǰ���ٶ�(-100��100)
  * @param[in]      turn_rate: ת������(-100��100)
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t car_manual_control(float forward_speed, float turn_rate)
{
    if (!is_initialized)
    {
        return 1;
    }

    /* ȷ�����ֶ�����ͨģʽ�²Ž��ܿ��� */
    if (car_info.mode == CAR_MODE_AUTO || car_info.mode == CAR_MODE_EMERGENCY)
    {
        return 2;
    }

    /* ����С���˶� */
    if (bsp_car_set_motion(forward_speed, turn_rate) != 0)
    {
        return 3;
    }

    /* ����״̬��Ϣ */
    car_info.forward_speed = forward_speed;
    car_info.turn_rate = turn_rate;

    return 0;
}

/**
  * @brief          ��ȡС��������Ϣ
  * @param[out]     info: ��Ϣ�ṹ��ָ��
  * @retval         0�ɹ�����0ʧ��
  */
uint8_t car_get_info(car_info_t* info)
{
    if (!is_initialized || info == NULL)
    {
        return 1;
    }

    /* ������Ϣ */
    *info = car_info;

    return 0;
}

/**
  * @brief          ����ֹͣС��
  * @param[in]      none
  * @retval         none
  */
void car_emergency_stop(void)
{
    /* ����ֹͣ���е�� */
    bsp_car_stop();

    /* ����״̬ */
    car_info.mode = CAR_MODE_EMERGENCY;
    car_info.state = CAR_STATE_STOP;
    car_info.forward_speed = 0.0f;
    car_info.turn_rate = 0.0f;
    car_info.left_speed = 0.0f;
    car_info.right_speed = 0.0f;
}

/**
  * @brief          ����ң����ģʽ�л�
  * @param[in]      none
  * @retval         none
  */
static void car_mode_switch(void)
{
    /* ���ң����������Ч�� */
    if (rc_data == NULL)
    {
        return;
    }

    /* ����ֹͣͨ����� */
    if (switch_is_up(rc_data[CAR_EMERGENCY_CHANNEL]))
    {
        car_emergency_stop();
        return;
    }

    /* ����ģʽ�л�ͨ������ģʽ */
    int16_t mode_channel = rc_data[CAR_MODE_SWITCH_CHANNEL];

    if (switch_is_down(mode_channel))
    {
        /* ��λ�� - ֹͣģʽ */
        car_set_mode(CAR_MODE_STOP);
    }
    else if (switch_is_mid(mode_channel))
    {
        /* �м�λ�� - ����ģʽ */
        car_set_mode(CAR_MODE_NORMAL);
    }
    else if (switch_is_up(mode_channel))
    {
        /* ��λ�� - ��������ͨ������ģʽ */
        if (rc_data[CAR_FORWARD_CHANNEL] > 300)
        {
            /* ����ģʽ */
            car_set_mode(CAR_MODE_SPORT);
        }
        else if (rc_data[CAR_FORWARD_CHANNEL] < -300)
        {
            /* ����ģʽ */
            car_set_mode(CAR_MODE_ECO);
        }
        else
        {
            /* Ĭ��Ϊ����ģʽ */
            car_set_mode(CAR_MODE_NORMAL);
        }
    }
}

/**
  * @brief          ����ң�������Ƹ���
  * @param[in]      none
  * @retval         none
  */
static void car_rc_control_update(void)
{
    if (rc_data == NULL ||
        car_info.mode == CAR_MODE_STOP ||
        car_info.mode == CAR_MODE_EMERGENCY ||
        car_info.mode == CAR_MODE_AUTO)
    {
        return;
    }