/**
  ******************************************************************************
  * @file       bsp_fan.c
  * @brief      ���PWM����BSP��
  * @note       ����STM32 HAL�⣬֧�ֶ������ƺ�ƽ��ת�ٱ仯
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

#include "bsp_fan.h"

/* ��̬����������� */
static fan_pwm_t fan_pwm[FAN_NUM];

/* �����ǰ״̬ */
static float fan_current_speed[FAN_NUM] = {0.0f};   // ��ǰת�ٰٷֱ�
static float fan_target_speed[FAN_NUM] = {0.0f};    // Ŀ��ת�ٰٷֱ�
static fan_ctrl_mode_e fan_ctrl_mode[FAN_NUM] = {FAN_CTRL_MANUAL}; // ����ģʽ

/**
  * @brief          PWMֵת��Ϊ�Ƚ�ֵ
  * @param[in]      htim: ��ʱ�����ָ��
  * @param[in]      percent: �ٷֱ�(0-100)
  * @retval         ��ʱ���Ƚ�ֵ
  */
static uint32_t fan_percent_to_compare(TIM_HandleTypeDef* htim, float percent)
{
    uint32_t period = htim->Init.Period;
    
    /* ���ưٷֱȷ�Χ */
    if (percent > 100.0f)
    {
        percent = 100.0f;
    }
    else if (percent < 0.0f)
    {
        percent = 0.0f;
    }
    
    /* ����Ƚ�ֵ */
    return (uint32_t)((percent / 100.0f) * period);
}

/**
  * @brief          ���BSP��ʼ��
  * @param[in]      htim: ���PWM��ʱ�����ָ��
  * @param[in]      channel: ���PWMͨ��
  * @retval         ����0��ʾ��ʼ���ɹ�����0��ʾʧ��
  */
uint8_t bsp_fan_init(TIM_HandleTypeDef* htim, uint32_t channel)
{
    /* ������� */
    if (htim == NULL)
    {
        return 1;
    }
    
    /* ����PWMʱ�� */
    uint32_t prescaler = HAL_RCC_GetSysClockFreq() / 2 / FAN_PWM_TIM_FREQ - 1;
    uint32_t period = FAN_PWM_PERIOD - 1;
    
    /* ���ö�ʱ���������� */
    htim->Init.Prescaler = prescaler;
    htim->Init.Period = period;
    HAL_TIM_Base_Init(htim);
    
    /* ����PWMģʽ */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    
    /* ����PWM��� */
    HAL_TIM_PWM_Start(htim, channel);
    
    /* �������� */
    fan_pwm[FAN_MAIN_INDEX].htim = htim;
    fan_pwm[FAN_MAIN_INDEX].channel = channel;
    fan_pwm[FAN_MAIN_INDEX].is_active = 1;
    
    /* ���ó�ʼ�ٶ�Ϊ0 */
    bsp_fan_set_speed(FAN_MAIN_INDEX, FAN_SPEED_DEFAULT);
    
    return 0;
}

/**
  * @brief          ���÷��ת�ٰٷֱ�
  * @param[in]      fan_index: �������
  * @param[in]      percent: ת�ٰٷֱ�(0-100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_fan_set_speed(uint8_t fan_index, float percent)
{
    /* ������� */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* ���Ʒ�Χ */
    if (percent > FAN_SPEED_MAX)
    {
        percent = FAN_SPEED_MAX;
    }
    else if (percent < FAN_SPEED_MIN)
    {
        percent = FAN_SPEED_MIN;
    }
    
    /* ���·��״̬ */
    fan_current_speed[fan_index] = percent;
    fan_target_speed[fan_index] = percent;
    fan_ctrl_mode[fan_index] = FAN_CTRL_MANUAL;
    
    /* ����PWM�Ƚ�ֵ */
    uint32_t compare = fan_percent_to_compare(fan_pwm[fan_index].htim, percent);
    
    /* ����PWM��� */
    __HAL_TIM_SET_COMPARE(fan_pwm[fan_index].htim, fan_pwm[fan_index].channel, compare);
    
    return 0;
}

/**
  * @brief          ��ȡ�����ǰת�ٰٷֱ�
  * @param[in]      fan_index: �������
  * @retval         ���ת�ٰٷֱ�(0-100)
  */
float bsp_fan_get_speed(uint8_t fan_index)
{
    if (fan_index >= FAN_NUM)
    {
        return 0.0f;
    }
    
    return fan_current_speed[fan_index];
}

/**
  * @brief          ���÷��Ŀ��ת�ٲ�����ƽ������
  * @param[in]      fan_index: �������
  * @param[in]      target_percent: Ŀ��ת�ٰٷֱ�(0-100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_fan_set_target_speed(uint8_t fan_index, float target_percent)
{
    /* ������� */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* ���Ʒ�Χ */
    if (target_percent > FAN_SPEED_MAX)
    {
        target_percent = FAN_SPEED_MAX;
    }
    else if (target_percent < FAN_SPEED_MIN)
    {
        target_percent = FAN_SPEED_MIN;
    }
    
    /* ����Ŀ���ٶ� */
    fan_target_speed[fan_index] = target_percent;
    fan_ctrl_mode[fan_index] = FAN_CTRL_AUTO;
    
    return 0;
}

/**
  * @brief          ����ٶȿ��ƴ�������Ӧ��ѭ���е�����ʵ��ƽ������
  * @param[in]      none
  * @retval         none
  */
void bsp_fan_speed_ctrl_update(void)
{
    for (uint8_t i = 0; i < FAN_NUM; i++)
    {
        /* ֻ�����Զ�����ģʽ */
        if (fan_ctrl_mode[i] != FAN_CTRL_AUTO || !fan_pwm[i].is_active)
        {
            continue;
        }
        
        /* ƽ�������ٶ� */
        if (fan_current_speed[i] < fan_target_speed[i])
        {
            fan_current_speed[i] += FAN_RAMP_RATE;
            if (fan_current_speed[i] > fan_target_speed[i])
            {
                fan_current_speed[i] = fan_target_speed[i];
            }
        }
        else if (fan_current_speed[i] > fan_target_speed[i])
        {
            fan_current_speed[i] -= FAN_RAMP_RATE;
            if (fan_current_speed[i] < fan_target_speed[i])
            {
                fan_current_speed[i] = fan_target_speed[i];
            }
        }
        
        /* ����PWM��� */
        uint32_t compare = fan_percent_to_compare(fan_pwm[i].htim, fan_current_speed[i]);
        __HAL_TIM_SET_COMPARE(fan_pwm[i].htim, fan_pwm[i].channel, compare);
    }
}

/**
  * @brief          ң�������Ʒ��
  * @param[in]      fan_index: �������
  * @param[in]      rc_value: ң����ֵ(-660��660)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_fan_control_by_rc(uint8_t fan_index, int16_t rc_value)
{
    /* ������� */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* Ӧ������ */
    if (rc_value > -FAN_RC_DEADBAND && rc_value < FAN_RC_DEADBAND)
    {
        rc_value = 0;
    }
    
    /* ����Ŀ���ٶ� */
    float target_speed;
    if (rc_value <= 0)  // ͨ��ң��������Ϊ��ֵ
    {
        /* ��ң����ֵӳ�䵽����ٶ� */
        target_speed = (float)(-rc_value) / (-FAN_RC_MIN_VALUE) * FAN_SPEED_MAX;
    }
    else
    {
        target_speed = 0.0f;
    }
    
    /* ����Ŀ���ٶ� */
    fan_target_speed[fan_index] = target_speed;
    fan_ctrl_mode[fan_index] = FAN_CTRL_RC;
    
    /* ���ô���������ƽ������ */
    bsp_fan_speed_ctrl_update();
    
    return 0;
}

/**
  * @brief          ֹͣ���з��
  * @param[in]      none
  * @retval         none
  */
void bsp_fan_stop_all(void)
{
    for (uint8_t i = 0; i < FAN_NUM; i++)
    {
        if (fan_pwm[i].is_active)
        {
            bsp_fan_set_speed(i, FAN_SPEED_MIN);
        }
    }
}

/**
  * @brief          ��ȡ�����ʼ��״̬
  * @param[in]      fan_index: �������
  * @retval         ����1��ʾ�ѳ�ʼ����0��ʾδ��ʼ��
  */
uint8_t bsp_fan_is_initialized(uint8_t fan_index)
{
    if (fan_index >= FAN_NUM)
    {
        return 0;
    }
    
    return fan_pwm[fan_index].is_active;
}