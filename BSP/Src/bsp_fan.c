/**
  ******************************************************************************
  * @file       bsp_fan.c
  * @brief      风机PWM驱动BSP层
  * @note       基于STM32 HAL库，支持多风机控制和平滑转速变化
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. 调试中
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "bsp_fan.h"

/* 静态风机配置数组 */
static fan_pwm_t fan_pwm[FAN_NUM];

/* 风机当前状态 */
static float fan_current_speed[FAN_NUM] = {0.0f};   // 当前转速百分比
static float fan_target_speed[FAN_NUM] = {0.0f};    // 目标转速百分比
static fan_ctrl_mode_e fan_ctrl_mode[FAN_NUM] = {FAN_CTRL_MANUAL}; // 控制模式

/**
  * @brief          PWM值转换为比较值
  * @param[in]      htim: 定时器句柄指针
  * @param[in]      percent: 百分比(0-100)
  * @retval         定时器比较值
  */
static uint32_t fan_percent_to_compare(TIM_HandleTypeDef* htim, float percent)
{
    uint32_t period = htim->Init.Period;
    
    /* 限制百分比范围 */
    if (percent > 100.0f)
    {
        percent = 100.0f;
    }
    else if (percent < 0.0f)
    {
        percent = 0.0f;
    }
    
    /* 计算比较值 */
    return (uint32_t)((percent / 100.0f) * period);
}

/**
  * @brief          风机BSP初始化
  * @param[in]      htim: 风机PWM定时器句柄指针
  * @param[in]      channel: 风机PWM通道
  * @retval         返回0表示初始化成功，非0表示失败
  */
uint8_t bsp_fan_init(TIM_HandleTypeDef* htim, uint32_t channel)
{
    /* 参数检查 */
    if (htim == NULL)
    {
        return 1;
    }
    
    /* 配置PWM时钟 */
    uint32_t prescaler = HAL_RCC_GetSysClockFreq() / 2 / FAN_PWM_TIM_FREQ - 1;
    uint32_t period = FAN_PWM_PERIOD - 1;
    
    /* 配置定时器基本参数 */
    htim->Init.Prescaler = prescaler;
    htim->Init.Period = period;
    HAL_TIM_Base_Init(htim);
    
    /* 配置PWM模式 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(htim, channel);
    
    /* 保存配置 */
    fan_pwm[FAN_MAIN_INDEX].htim = htim;
    fan_pwm[FAN_MAIN_INDEX].channel = channel;
    fan_pwm[FAN_MAIN_INDEX].is_active = 1;
    
    /* 设置初始速度为0 */
    bsp_fan_set_speed(FAN_MAIN_INDEX, FAN_SPEED_DEFAULT);
    
    return 0;
}

/**
  * @brief          设置风机转速百分比
  * @param[in]      fan_index: 风机索引
  * @param[in]      percent: 转速百分比(0-100)
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_fan_set_speed(uint8_t fan_index, float percent)
{
    /* 参数检查 */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* 限制范围 */
    if (percent > FAN_SPEED_MAX)
    {
        percent = FAN_SPEED_MAX;
    }
    else if (percent < FAN_SPEED_MIN)
    {
        percent = FAN_SPEED_MIN;
    }
    
    /* 更新风机状态 */
    fan_current_speed[fan_index] = percent;
    fan_target_speed[fan_index] = percent;
    fan_ctrl_mode[fan_index] = FAN_CTRL_MANUAL;
    
    /* 计算PWM比较值 */
    uint32_t compare = fan_percent_to_compare(fan_pwm[fan_index].htim, percent);
    
    /* 设置PWM输出 */
    __HAL_TIM_SET_COMPARE(fan_pwm[fan_index].htim, fan_pwm[fan_index].channel, compare);
    
    return 0;
}

/**
  * @brief          获取风机当前转速百分比
  * @param[in]      fan_index: 风机索引
  * @retval         风机转速百分比(0-100)
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
  * @brief          设置风机目标转速并启用平滑控制
  * @param[in]      fan_index: 风机索引
  * @param[in]      target_percent: 目标转速百分比(0-100)
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_fan_set_target_speed(uint8_t fan_index, float target_percent)
{
    /* 参数检查 */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* 限制范围 */
    if (target_percent > FAN_SPEED_MAX)
    {
        target_percent = FAN_SPEED_MAX;
    }
    else if (target_percent < FAN_SPEED_MIN)
    {
        target_percent = FAN_SPEED_MIN;
    }
    
    /* 更新目标速度 */
    fan_target_speed[fan_index] = target_percent;
    fan_ctrl_mode[fan_index] = FAN_CTRL_AUTO;
    
    return 0;
}

/**
  * @brief          风机速度控制处理函数，应在循环中调用以实现平滑控制
  * @param[in]      none
  * @retval         none
  */
void bsp_fan_speed_ctrl_update(void)
{
    for (uint8_t i = 0; i < FAN_NUM; i++)
    {
        /* 只处理自动控制模式 */
        if (fan_ctrl_mode[i] != FAN_CTRL_AUTO || !fan_pwm[i].is_active)
        {
            continue;
        }
        
        /* 平滑调整速度 */
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
        
        /* 更新PWM输出 */
        uint32_t compare = fan_percent_to_compare(fan_pwm[i].htim, fan_current_speed[i]);
        __HAL_TIM_SET_COMPARE(fan_pwm[i].htim, fan_pwm[i].channel, compare);
    }
}

/**
  * @brief          遥控器控制风机
  * @param[in]      fan_index: 风机索引
  * @param[in]      rc_value: 遥控器值(-660至660)
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_fan_control_by_rc(uint8_t fan_index, int16_t rc_value)
{
    /* 参数检查 */
    if (fan_index >= FAN_NUM || !fan_pwm[fan_index].is_active)
    {
        return 1;
    }
    
    /* 应用死区 */
    if (rc_value > -FAN_RC_DEADBAND && rc_value < FAN_RC_DEADBAND)
    {
        rc_value = 0;
    }
    
    /* 计算目标速度 */
    float target_speed;
    if (rc_value <= 0)  // 通常遥控器下拉为负值
    {
        /* 将遥控器值映射到风机速度 */
        target_speed = (float)(-rc_value) / (-FAN_RC_MIN_VALUE) * FAN_SPEED_MAX;
    }
    else
    {
        target_speed = 0.0f;
    }
    
    /* 设置目标速度 */
    fan_target_speed[fan_index] = target_speed;
    fan_ctrl_mode[fan_index] = FAN_CTRL_RC;
    
    /* 调用处理函数进行平滑控制 */
    bsp_fan_speed_ctrl_update();
    
    return 0;
}

/**
  * @brief          停止所有风机
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
  * @brief          获取风机初始化状态
  * @param[in]      fan_index: 风机索引
  * @retval         返回1表示已初始化，0表示未初始化
  */
uint8_t bsp_fan_is_initialized(uint8_t fan_index)
{
    if (fan_index >= FAN_NUM)
    {
        return 0;
    }
    
    return fan_pwm[fan_index].is_active;
}