/**
 * @file bsp_fan.c
 * @brief DJI C板风扇驱动BSP（仅闭环控制）
 * @details 使用TIM8_CH1输出PWM控制风扇，GPIO(PC6)输入测量FG反馈信号
 * @author Feiziben
 * @date 2025.03.20
 */
#include "bsp_fan.h"
#include <string.h>

/* 外部HAL定时器句柄 */
extern TIM_HandleTypeDef htim8;

/* 定义FG引脚 - PE9引脚 */
#define FAN_FG_GPIO_PORT       GPIOE
#define FAN_FG_GPIO_PIN        GPIO_PIN_9
#define FAN_PWM_CHANNEL        TIM_CHANNEL_1  // TIM8_CH1

/* PID默认参数 */
#define PID_KP_DEFAULT         1.0f    // 比例系数
#define PID_KI_DEFAULT         0.2f    // 积分系数
#define PID_KD_DEFAULT         0.01f   // 微分系数

#define PID_OUTPUT_MIN         FAN_START_DUTY  // 最小输出值（风扇启动占空比）
#define PID_INTEGRAL_LIMIT     2000    // 积分限幅

/* 私有变量 */
static uint16_t fan_pwm_value = 0;
static uint16_t fan_rpm = 0;
static uint16_t fan_target_rpm = 0;
static uint8_t fan_running = 0;

/* PID控制器 */
static fan_pid_t fan_pid;

/* FG信号测量相关变量 */
static uint32_t fg_last_time = 0;
static uint32_t fg_period_ms = 0;
static uint32_t fg_edge_count = 0;
static uint32_t fg_filter_buffer[FAN_FG_FILTER_SIZE] = {0};
static uint8_t fg_filter_index = 0;
static uint32_t fg_last_update = 0;

/**
 * @brief PID控制器初始化
 * @param 无
 * @return 无
 */
static void pid_init(void)
{
    /* 设置PID参数 */
    fan_pid.kp = PID_KP_DEFAULT;
    fan_pid.ki = PID_KI_DEFAULT;
    fan_pid.kd = PID_KD_DEFAULT;
    
    /* 重置PID状态 */
    fan_pid.error = 0.0f;
    fan_pid.error_sum = 0.0f;
    fan_pid.error_last = 0.0f;
    fan_pid.output = 0.0f;
    
    /* 设置输出限幅 */
    fan_pid.output_max = PID_OUTPUT_MAX;
    fan_pid.output_min = PID_OUTPUT_MIN;
}

/**
 * @brief PID计算
 * @param[in] target 目标值
 * @param[in] current 当前值
 * @return PID输出值
 */
static float pid_calculate(float target, float current)
{
    /* 计算误差 */
    fan_pid.error = target - current;
    
    /* 累加误差（积分项） */
    fan_pid.error_sum += fan_pid.error;
    
    /* 积分限幅，防止积分饱和 */
    if (fan_pid.error_sum > PID_INTEGRAL_LIMIT) {
        fan_pid.error_sum = PID_INTEGRAL_LIMIT;
    } else if (fan_pid.error_sum < -PID_INTEGRAL_LIMIT) {
        fan_pid.error_sum = -PID_INTEGRAL_LIMIT;
    }
    
    /* PID计算 */
    fan_pid.output = fan_pid.kp * fan_pid.error + 
                     fan_pid.ki * fan_pid.error_sum + 
                     fan_pid.kd * (fan_pid.error - fan_pid.error_last);
    
    /* 保存当前误差用于下次计算 */
    fan_pid.error_last = fan_pid.error;
    
    /* 输出限幅 */
    if (fan_pid.output > fan_pid.output_max) {
        fan_pid.output = fan_pid.output_max;
    } else if (fan_pid.output < fan_pid.output_min && target > 0) {
        /* 只有当目标值大于0时才限制最小值 */
        fan_pid.output = fan_pid.output_min;
    } else if (fan_pid.output < 0 || target == 0) {
        /* 如果目标值为0或输出小于0，则输出0 */
        fan_pid.output = 0;
    }
    
    return fan_pid.output;
}

/**
 * @brief 风扇初始化
 * @param 无
 * @return 无
 */
void fan_init(void)
{
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim8, FAN_PWM_CHANNEL);
    
    /* 设置初始PWM为0（风扇停止） */
    __HAL_TIM_SET_COMPARE(&htim8, FAN_PWM_CHANNEL, 0);
    
    /* 配置FG引脚为输入，启用内部上拉 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FAN_FG_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // 上升沿触发中断
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 启用内部上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FAN_FG_GPIO_PORT, &GPIO_InitStruct);
    
    /* 确保EXTI中断线已启用 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);  // PC6对应EXTI9_5_IRQn
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    /* 初始化PID控制器 */
    pid_init();
    
    /* 初始化变量 */
    fan_pwm_value = 0;
    fan_rpm = 0;
    fan_target_rpm = 0;
    fan_running = 0;
    
    fg_last_time = 0;
    fg_period_ms = 0;
    fg_edge_count = 0;
    fg_last_update = HAL_GetTick();
    
    /* 初始化滤波缓冲区 */
    memset(fg_filter_buffer, 0, sizeof(fg_filter_buffer));
}

/**
 * @brief 计算风扇转速（移动平均滤波）
 * @param period_ms FG信号周期（毫秒）
 * @return 风扇转速（RPM）
 */
static uint16_t calculate_fan_rpm(uint32_t period_ms)
{
    /* 记录当前测量值到滤波缓冲区 */
    fg_filter_buffer[fg_filter_index] = period_ms;
    fg_filter_index = (fg_filter_index + 1) % FAN_FG_FILTER_SIZE;
    
    /* 计算平均周期 */
    uint32_t sum = 0;
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < FAN_FG_FILTER_SIZE; i++) {
        if (fg_filter_buffer[i] > 0) {
            sum += fg_filter_buffer[i];
            valid_count++;
        }
    }
    
    /* 如果没有有效数据，返回0 */
    if (valid_count == 0) {
        return 0;
    }
    
    /* 计算平均周期 */
    uint32_t avg_period_ms = sum / valid_count;
    
    /* 防止除零错误 */
    if (avg_period_ms == 0) {
        return 0;
    }
    
    /* 计算频率（Hz）= 1000 / 周期（ms）*/
    uint32_t freq_hz = 1000 / avg_period_ms;
    
    /* 转换为RPM: RPM = 频率 * 60 */
    return (uint16_t)(freq_hz * 60);
}

/**
 * @brief 直接设置PWM值
 * @param[in] pwm_value PWM值(0-FAN_MAX_DUTY)
 * @return 无
 */
static void set_pwm(uint16_t pwm_value)
{
    /* 限制PWM值范围 */
    if (pwm_value > FAN_MAX_DUTY) {
        pwm_value = FAN_MAX_DUTY;
    }
    
    /* 更新PWM输出 */
    __HAL_TIM_SET_COMPARE(&htim8, FAN_PWM_CHANNEL, pwm_value);
    fan_pwm_value = pwm_value;
    
    /* 更新运行状态 */
    fan_running = (pwm_value > 0) ? 1 : 0;
}

/**
 * @brief 设置风扇目标转速
 * @param[in] target_rpm 目标转速(0-FAN_RPM_MAX)
 * @return 无
 */
void fan_set_rpm(uint16_t target_rpm)
{
    /* 限制目标转速范围 */
    if (target_rpm > FAN_RPM_MAX) {
        target_rpm = FAN_RPM_MAX;
    }
    
    /* 更新目标转速 */
    fan_target_rpm = target_rpm;
    
    /* 重置PID积分项，防止突变 */
    fan_pid.error_sum = 0.0f;
    
    /* 如果目标转速为0，直接停止风扇 */
    if (target_rpm == 0) {
        set_pwm(0);
    } else if (!fan_running) {
        /* 如果风扇当前未运行，以启动阈值启动风扇 */
        set_pwm(FAN_START_DUTY);
    }
}

/**
 * @brief 停止风扇
 * @param 无
 * @return 无
 */
void fan_stop(void)
{
    /* 设置PWM为0停止风扇 */
    set_pwm(0);
    
    /* 更新状态 */
    fan_target_rpm = 0;
    
    /* 重置PID控制器 */
    fan_pid.error = 0.0f;
    fan_pid.error_sum = 0.0f;
    fan_pid.error_last = 0.0f;
}

/**
 * @brief 获取风扇转速
 * @param 无
 * @return 风扇转速(RPM)
 */
uint16_t fan_get_rpm(void)
{
    return fan_rpm;
}

/**
 * @brief 获取风扇目标转速
 * @param 无
 * @return 目标转速(RPM)
 */
uint16_t fan_get_target_rpm(void)
{
    return fan_target_rpm;
}

/**
 * @brief 风扇FG信号GPIO外部中断回调
 * @note 需在HAL_GPIO_EXTI_Callback中调用
 * @param[in] GPIO_Pin 触发中断的GPIO引脚
 * @return 无
 */
void fan_fg_exti_callback(uint16_t GPIO_Pin)
{
    /* 检查是否是FG引脚触发的中断 */
    if (GPIO_Pin == FAN_FG_GPIO_PIN) {
        uint32_t current_time = HAL_GetTick();
        
        /* 如果不是首次触发，计算周期 */
        if (fg_last_time > 0) {
            fg_period_ms = current_time - fg_last_time;
            
            /* 仅处理合理范围内的周期值，避免异常值 */
            if (fg_period_ms > 0 && fg_period_ms < 1000) {
                /* 更新RPM值 */
                fan_rpm = calculate_fan_rpm(fg_period_ms);
                fg_edge_count++;
            }
        }
        
        /* 更新时间戳 */
        fg_last_time = current_time;
        fg_last_update = current_time;
    }
}

/**
 * @brief 风扇控制周期处理
 * @note 需在主循环或定时任务中每10ms调用一次
 * @param 无
 * @return 无
 */
void fan_process(void)
{
    static uint32_t last_control_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    /* 检查FG信号是否超时 */
    if (current_time - fg_last_update > FAN_FG_TIMEOUT_MS) {
        /* 如果超过超时时间没有接收到脉冲，认为风扇停止 */
        fg_last_time = 0;
        fan_rpm = 0;
        
        /* 重置滤波缓冲区 */
        memset(fg_filter_buffer, 0, sizeof(fg_filter_buffer));
    }
    
    /* 检查是否到达控制周期 */
    if (current_time - last_control_time >= FAN_CONTROL_PERIOD_MS) {
        last_control_time = current_time;
        
        /* 执行PID控制 */
        if (fan_target_rpm > 0) {
            /* 计算PID输出 */
            float pid_output = pid_calculate((float)fan_target_rpm, (float)fan_rpm);
            
            /* 更新PWM值 */
            set_pwm((uint16_t)pid_output);
        }
    }
}

