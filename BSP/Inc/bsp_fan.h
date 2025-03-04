/**
  ******************************************************************************
  * @file       bsp_fan.h
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
  
#ifndef BSP_FAN_H
#define BSP_FAN_H

#include "main.h"

/* 风机数量和通道定义 */
#define FAN_NUM                 1        // 风机数量
#define FAN_MAIN_INDEX          0        // 主风机索引

/* PWM配置 */
#define FAN_PWM_TIM_FREQ        1000000  // 定时器频率 1MHz
#define FAN_PWM_FREQ            50       // PWM频率 50Hz
#define FAN_PWM_RESOLUTION      10000    // PWM精度(0-10000)
#define FAN_PWM_PERIOD          (FAN_PWM_TIM_FREQ / FAN_PWM_FREQ) // PWM周期

/* 转速范围和控制参数 */
#define FAN_SPEED_MAX           100      // 最大转速百分比
#define FAN_SPEED_MIN           0        // 最小转速百分比
#define FAN_SPEED_DEFAULT       0        // 默认转速百分比
#define FAN_RAMP_RATE           2        // 每次控制转速变化率(%)

/* 遥控器控制参数 */
#define FAN_RC_DEADBAND         5        // 遥控器死区
#define FAN_RC_MIN_VALUE        -660     // 遥控器最小值
#define FAN_RC_MAX_VALUE        660      // 遥控器最大值

/* 风机PWM通道配置 */
typedef struct {
    TIM_HandleTypeDef* htim;     // 定时器句柄指针
    uint32_t           channel;  // 定时器通道
    uint8_t            is_active; // 是否激活
} fan_pwm_t;

/* 风机控制模式枚举 */
typedef enum {
    FAN_CTRL_MANUAL = 0,        // 手动控制模式
    FAN_CTRL_RC,                // 遥控器控制模式
    FAN_CTRL_AUTO,              // 自动控制模式
} fan_ctrl_mode_e;

/* 函数声明 */
/**
  * @brief          风机BSP初始化
  * @param[in]      htim: 风机PWM定时器句柄指针
  * @param[in]      channel: 风机PWM通道
  * @retval         返回0表示初始化成功，非0表示失败
  */
extern uint8_t bsp_fan_init(TIM_HandleTypeDef* htim, uint32_t channel);

/**
  * @brief          设置风机转速百分比
  * @param[in]      fan_index: 风机索引
  * @param[in]      percent: 转速百分比(0-100)
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_fan_set_speed(uint8_t fan_index, float percent);

/**
  * @brief          获取风机当前转速百分比
  * @param[in]      fan_index: 风机索引
  * @retval         风机转速百分比(0-100)
  */
extern float bsp_fan_get_speed(uint8_t fan_index);

/**
  * @brief          设置风机目标转速并启用平滑控制
  * @param[in]      fan_index: 风机索引
  * @param[in]      target_percent: 目标转速百分比(0-100)
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_fan_set_target_speed(uint8_t fan_index, float target_percent);

/**
  * @brief          风机速度控制处理函数，应在循环中调用以实现平滑控制
  * @param[in]      none
  * @retval         none
  */
extern void bsp_fan_speed_ctrl_update(void);

/**
  * @brief          遥控器控制风机
  * @param[in]      fan_index: 风机索引
  * @param[in]      rc_value: 遥控器值(-660至660)
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_fan_control_by_rc(uint8_t fan_index, int16_t rc_value);

/**
  * @brief          停止所有风机
  * @param[in]      none
  * @retval         none
  */
extern void bsp_fan_stop_all(void);

/**
  * @brief          获取风机初始化状态
  * @param[in]      fan_index: 风机索引
  * @retval         返回1表示已初始化，0表示未初始化
  */
extern uint8_t bsp_fan_is_initialized(uint8_t fan_index);

#endif /* BSP_FAN_H */