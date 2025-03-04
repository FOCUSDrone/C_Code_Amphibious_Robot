/**
  ******************************************************************************
  * @file       bsp_car.c
  * @brief      小车电机驱动BSP层，支持差速转向
  * @note       基于STM32 HAL库，使用PWM控制电调
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

#include "bsp_car.h"
#include <math.h>

/* 静态电机配置数组 */
static motor_pwm_t motor_pwm[CAR_MOTOR_NUM];

/* 小车当前状态 */
static float motor_current_speed[CAR_MOTOR_NUM] = {0.0f, 0.0f}; // 当前电机速度
static float motor_target_speed[CAR_MOTOR_NUM] = {0.0f, 0.0f};  // 目标电机速度
static car_ctrl_mode_e car_ctrl_mode = CAR_CTRL_MANUAL;          // 控制模式
static float car_forward_speed = 0.0f;    // 小车前进速度
static float car_turn_rate = 0.0f;        // 小车转向速率
static float car_target_forward = 0.0f;   // 目标前进速度
static float car_target_turn = 0.0f;      // 目标转向速率

/**
  * @brief          速度值转换为PWM值
  * @param[in]      speed: 速度值(-100至100)
  * @param[in]      motor_dir: 电机方向
  * @retval         PWM值(1000-2000)
  */
static uint32_t motor_speed_to_pwm(float speed, motor_dir_e motor_dir)
{
    /* 限制速度范围 */
    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < -100.0f)
    {
        speed = -100.0f;
    }
    
    /* 应用电机方向反转 */
    if (motor_dir == MOTOR_DIR_REVERSE)
    {
        speed = -speed;
    }
    
    /* 计算PWM值 */
    uint32_t pwm;
    if (speed > 0)
    {
        pwm = CAR_PWM_NEUTRAL + (uint32_t)((speed / 100.0f) * (CAR_PWM_FORWARD_MAX - CAR_PWM_NEUTRAL));
    }
    else if (speed < 0)
    {
        pwm = CAR_PWM_NEUTRAL - (uint32_t)(((-speed) / 100.0f) * (CAR_PWM_NEUTRAL - CAR_PWM_BACKWARD_MAX));
    }
    else
    {
        pwm = CAR_PWM_NEUTRAL;
    }
    
    return pwm;
}

/**
  * @brief          PWM值转换为微秒值
  * @param[in]      htim: 定时器句柄指针
  * @param[in]      us_value: 微秒值
  * @retval         定时器比较值
  */
static uint32_t pwm_us_to_compare(TIM_HandleTypeDef* htim, uint32_t us_value)
{
    uint32_t period = htim->Init.Period;
    uint32_t freq = CAR_PWM_TIM_FREQ;
    
    /* 计算比较值 */
    return (uint32_t)((float)us_value * (float)freq / 1000000.0f);
}

/**
  * @brief          小车BSP初始化
  * @param[in]      left_motor: 左电机定时器句柄
  * @param[in]      left_channel: 左电机PWM通道
  * @param[in]      right_motor: 右电机定时器句柄
  * @param[in]      right_channel: 右电机PWM通道
  * @retval         返回0表示初始化成功，非0表示失败
  */
uint8_t bsp_car_init(TIM_HandleTypeDef* left_motor, uint32_t left_channel,
                   TIM_HandleTypeDef* right_motor, uint32_t right_channel)
{
    /* 参数检查 */
    if (left_motor == NULL || right_motor == NULL)
    {
        return 1;
    }
    
    /* 配置PWM时钟 */
    uint32_t prescaler = HAL_RCC_GetSysClockFreq() / 2 / CAR_PWM_TIM_FREQ - 1;
    uint32_t period = CAR_PWM_PERIOD - 1;
    
    /* 配置左电机定时器 */
    left_motor->Init.Prescaler = prescaler;
    left_motor->Init.Period = period;
    HAL_TIM_Base_Init(left_motor);
    
    /* 配置右电机定时器（如果与左电机不同） */
    if (right_motor != left_motor)
    {
        right_motor->Init.Prescaler = prescaler;
        right_motor->Init.Period = period;
        HAL_TIM_Base_Init(right_motor);
    }
    
    /* 配置PWM模式 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_us_to_compare(left_motor, CAR_PWM_NEUTRAL);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* 配置左电机PWM */
    HAL_TIM_PWM_ConfigChannel(left_motor, &sConfigOC, left_channel);
    HAL_TIM_PWM_Start(left_motor, left_channel);
    
    /* 配置右电机PWM */
    sConfigOC.Pulse = pwm_us_to_compare(right_motor, CAR_PWM_NEUTRAL);
    HAL_TIM_PWM_ConfigChannel(right_motor, &sConfigOC, right_channel);
    HAL_TIM_PWM_Start(right_motor, right_channel);
    
    /* 保存配置 */
    motor_pwm[CAR_LEFT_MOTOR_INDEX].htim = left_motor;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].channel = left_channel;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].dir = MOTOR_DIR_NORMAL;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].is_active = 1;
    
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].htim = right_motor;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].channel = right_channel;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].dir = MOTOR_DIR_NORMAL;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].is_active = 1;
    
    /* 设置初始状态为停止 */
    bsp_car_stop();
    
    return 0;
}

/**
  * @brief          设置电机方向
  * @param[in]      motor_index: 电机索引
  * @param[in]      dir: 电机方向
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_car_set_motor_direction(uint8_t motor_index, motor_dir_e dir)
{
    /* 参数检查 */
    if (motor_index >= CAR_MOTOR_NUM || !motor_pwm[motor_index].is_active)
    {
        return 1;
    }
    
    /* 设置方向 */
    motor_pwm[motor_index].dir = dir;
    
    /* 更新电机PWM */
    uint32_t pwm = motor_speed_to_pwm(motor_current_speed[motor_index], dir);
    uint32_t compare = pwm_us_to_compare(motor_pwm[motor_index].htim, pwm);
    __HAL_TIM_SET_COMPARE(motor_pwm[motor_index].htim, motor_pwm[motor_index].channel, compare);
    
    return 0;
}

/**
  * @brief          设置电机速度
  * @param[in]      motor_index: 电机索引
  * @param[in]      speed: 速度值(-100至100)，正值前进，负值后退
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_car_set_motor_speed(uint8_t motor_index, float speed)
{
    /* 参数检查 */
    if (motor_index >= CAR_MOTOR_NUM || !motor_pwm[motor_index].is_active)
    {
        return 1;
    }
    
    /* 限制速度范围 */
    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < -100.0f)
    {
        speed = -100.0f;
    }
    
    /* 更新电机状态 */
    motor_current_speed[motor_index] = speed;
    motor_target_speed[motor_index] = speed;
    
    /* 计算PWM值 */
    uint32_t pwm = motor_speed_to_pwm(speed, motor_pwm[motor_index].dir);
    uint32_t compare = pwm_us_to_compare(motor_pwm[motor_index].htim, pwm);
    
    /* 设置PWM输出 */
    __HAL_TIM_SET_COMPARE(motor_pwm[motor_index].htim, motor_pwm[motor_index].channel, compare);
    
    return 0;
}

/**
  * @brief          获取电机当前速度
  * @param[in]      motor_index: 电机索引
  * @retval         速度值(-100至100)
  */
float bsp_car_get_motor_speed(uint8_t motor_index)
{
    if (motor_index >= CAR_MOTOR_NUM)
    {
        return 0.0f;
    }
    
    return motor_current_speed[motor_index];
}

/**
  * @brief          设置小车运动
  * @param[in]      forward_speed: 前进速度(-100至100)
  * @param[in]      turn_rate: 转向速率(-100至100)，正值右转，负值左转
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_car_set_motion(float forward_speed, float turn_rate)
{
    /* 限制速度范围 */
    if (forward_speed > 100.0f)
    {
        forward_speed = 100.0f;
    }
    else if (forward_speed < -100.0f)
    {
        forward_speed = -100.0f;
    }
    
    /* 限制转向范围 */
    if (turn_rate > 100.0f)
    {
        turn_rate = 100.0f;
    }
    else if (turn_rate < -100.0f)
    {
        turn_rate = -100.0f;
    }
    
    /* 计算左右电机速度 */
    float left_speed = forward_speed + turn_rate;
    float right_speed = forward_speed - turn_rate;
    
    /* 限制最大速度 */
    float max_speed = fmaxf(fabsf(left_speed), fabsf(right_speed));
    if (max_speed > 100.0f)
    {
        float scale = 100.0f / max_speed;
        left_speed *= scale;
        right_speed *= scale;
    }
    
    /* 更新小车状态 */
    car_forward_speed = forward_speed;
    car_turn_rate = turn_rate;
    car_target_forward = forward_speed;
    car_target_turn = turn_rate;
    car_ctrl_mode = CAR_CTRL_MANUAL;
    
    /* 设置电机速度 */
    bsp_car_set_motor_speed(CAR_LEFT_MOTOR_INDEX, left_speed);
    bsp_car_set_motor_speed(CAR_RIGHT_MOTOR_INDEX, right_speed);
    
    return 0;
}

/**
  * @brief          设置小车目标运动并启用平滑控制
  * @param[in]      target_forward: 目标前进速度(-100至100)
  * @param[in]      target_turn: 目标转向速率(-100至100)
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_car_set_target_motion(float target_forward, float target_turn)
{
    /* 限制速度范围 */
    if (target_forward > 100.0f)
    {
        target_forward = 100.0f;
    }
    else if (target_forward < -100.0f)
    {
        target_forward = -100.0f;
    }
    
    /* 限制转向范围 */
    if (target_turn > 100.0f)
    {
        target_turn = 100.0f;
    }
    else if (target_turn < -100.0f)
    {
        target_turn = -100.0f;
    }
    
    /* 更新目标状态 */
    car_target_forward = target_forward;
    car_target_turn = target_turn;
    car_ctrl_mode = CAR_CTRL_AUTO;
    
    return 0;
}

/**
  * @brief          小车运动控制处理函数，应在循环中调用以实现平滑控制
  * @param[in]      none
  * @retval         none
  */
void bsp_car_motion_ctrl_update(void)
{
    /* 仅在自动控制模式下工作 */
    if (car_ctrl_mode != CAR_CTRL_AUTO)
    {
        return;
    }
    
    /* 平滑调整前进速度 */
    if (car_forward_speed < car_target_forward)
    {
        car_forward_speed += CAR_SPEED_RAMP_RATE;
        if (car_forward_speed > car_target_forward)
        {
            car_forward_speed = car_target_forward;
        }
    }
    else if (car_forward_speed > car_target_forward)
    {
        car_forward_speed -= CAR_SPEED_RAMP_RATE;
        if (car_forward_speed < car_target_forward)
        {
            car_forward_speed = car_target_forward;
        }
    }
    
    /* 平滑调整转向速率 */
    if (car_turn_rate < car_target_turn)
    {
        car_turn_rate += CAR_SPEED_RAMP_RATE;
        if (car_turn_rate > car_target_turn)
        {
            car_turn_rate = car_target_turn;
        }
    }
    else if (car_turn_rate > car_target_turn)
    {
        car_turn_rate -= CAR_SPEED_RAMP_RATE;
        if (car_turn_rate < car_target_turn)
        {
            car_turn_rate = car_target_turn;
        }
    }
    
    /* 计算左右电机速度 */
    float left_speed = car_forward_speed + car_turn_rate;
    float right_speed = car_forward_speed - car_turn_rate;
    
    /* 限制最大速度 */
    float max_speed = fmaxf(fabsf(left_speed), fabsf(right_speed));
    if (max_speed > 100.0f)
    {
        float scale = 100.0f / max_speed;
        left_speed *= scale;
        right_speed *= scale;
    }
    
    /* 更新电机速度 */
    motor_target_speed[CAR_LEFT_MOTOR_INDEX] = left_speed;
    motor_target_speed[CAR_RIGHT_MOTOR_INDEX] = right_speed;
    
    /* 设置电机PWM */
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (!motor_pwm[i].is_active)
        {
            continue;
        }
        
        /* 平滑调整电机速度 */
        if (motor_current_speed[i] < motor_target_speed[i])
        {
            motor_current_speed[i] += CAR_SPEED_RAMP_RATE;
            if (motor_current_speed[i] > motor_target_speed[i])
            {
                motor_current_speed[i] = motor_target_speed[i];
            }
        }
        else if (motor_current_speed[i] > motor_target_speed[i])
        {
            motor_current_speed[i] -= CAR_SPEED_RAMP_RATE;
            if (motor_current_speed[i] < motor_target_speed[i])
            {
                motor_current_speed[i] = motor_target_speed[i];
            }
        }
        
        /* 更新PWM输出 */
        uint32_t pwm = motor_speed_to_pwm(motor_current_speed[i], motor_pwm[i].dir);
        uint32_t compare = pwm_us_to_compare(motor_pwm[i].htim, pwm);
        __HAL_TIM_SET_COMPARE(motor_pwm[i].htim, motor_pwm[i].channel, compare);
    }
}

/**
  * @brief          遥控器控制小车
  * @param[in]      forward_ch: 前进通道值(-660至660)
  * @param[in]      turn_ch: 转向通道值(-660至660)
  * @retval         返回0表示成功，非0表示失败
  */
uint8_t bsp_car_control_by_rc(int16_t forward_ch, int16_t turn_ch)
{
    /* 应用死区 */
    if (forward_ch > -CAR_RC_DEADBAND && forward_ch < CAR_RC_DEADBAND)
    {
        forward_ch = 0;
    }
    if (turn_ch > -CAR_RC_DEADBAND && turn_ch < CAR_RC_DEADBAND)
    {
        turn_ch = 0;
    }
    
    /* 将遥控器值映射到小车控制值 */
    float target_forward = (float)forward_ch / (float)CAR_RC_MAX_VALUE * 100.0f;
    float target_turn = (float)turn_ch / (float)CAR_RC_MAX_VALUE * 100.0f;
    
    /* 设置控制模式为遥控器控制 */
    car_ctrl_mode = CAR_CTRL_RC;
    
    /* 设置目标运动 */
    return bsp_car_set_target_motion(target_forward, target_turn);
}

/**
  * @brief          停止小车
  * @param[in]      none
  * @retval         none
  */
void bsp_car_stop(void)
{
    /* 设置所有电机速度为0 */
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (motor_pwm[i].is_active)
        {
            bsp_car_set_motor_speed(i, 0.0f);
        }
    }
    
    /* 更新小车状态 */
    car_forward_speed = 0.0f;
    car_turn_rate = 0.0f;
    car_target_forward = 0.0f;
    car_target_turn = 0.0f;
}

/**
  * @brief          获取小车初始化状态
  * @retval         返回1表示已初始化，0表示未初始化
  */
uint8_t bsp_car_is_initialized(void)
{
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (!motor_pwm[i].is_active)
        {
            return 0;
        }
    }
    
    return 1;
}