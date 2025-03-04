/**
  ******************************************************************************
  * @file       bsp_car.h
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
  
#ifndef BSP_CAR_H
#define BSP_CAR_H

#include "main.h"

/* 电机数量和配置 */
#define CAR_MOTOR_NUM            2        // 电机数量
#define CAR_LEFT_MOTOR_INDEX     0        // 左电机索引
#define CAR_RIGHT_MOTOR_INDEX    1        // 右电机索引

/* PWM配置 */
#define CAR_PWM_TIM_FREQ         1000000  // 定时器频率 1MHz
#define CAR_PWM_FREQ             50       // PWM频率 50Hz (标准电调)
#define CAR_PWM_PERIOD           (CAR_PWM_TIM_FREQ / CAR_PWM_FREQ) // PWM周期

/* 电调PWM参数(标准电调为1000-2000us) */
#define CAR_PWM_NEUTRAL          1500     // 中间值（电机停止）
#define CAR_PWM_FORWARD_MAX      2000     // 最大前进值
#define CAR_PWM_BACKWARD_MAX     1000     // 最大后退值
#define CAR_PWM_DEADBAND         50       // 死区范围

/* 转向控制参数 */
#define CAR_TURN_RATE            0.5f     // 转向灵敏度(0-1)
#define CAR_SPEED_RAMP_RATE      10       // 速度渐变率

/* 遥控器控制参数 */
#define CAR_RC_DEADBAND          5        // 遥控器死区
#define CAR_RC_MAX_VALUE         660      // 遥控器最大值
#define CAR_RC_MIN_VALUE         -660     // 遥控器最小值

/* 电机方向定义 */
typedef enum {
    MOTOR_DIR_NORMAL = 0,        // 正常方向
    MOTOR_DIR_REVERSE = 1,       // 反向
} motor_dir_e;

/* 电机PWM通道配置 */
typedef struct {
    TIM_HandleTypeDef* htim;     // 定时器句柄指针
    uint32_t           channel;  // 定时器通道
    motor_dir_e        dir;      // 电机方向
    uint8_t            is_active; // 是否激活
} motor_pwm_t;

/* 小车控制模式枚举 */
typedef enum {
    CAR_CTRL_MANUAL = 0,         // 手动控制模式
    CAR_CTRL_RC,                 // 遥控器控制模式
    CAR_CTRL_AUTO,               // 自动控制模式
} car_ctrl_mode_e;

/* 函数声明 */
/**
  * @brief          小车BSP初始化
  * @param[in]      left_motor: 左电机定时器句柄
  * @param[in]      left_channel: 左电机PWM通道
  * @param[in]      right_motor: 右电机定时器句柄
  * @param[in]      right_channel: 右电机PWM通道
  * @retval         返回0表示初始化成功，非0表示失败
  */
extern uint8_t bsp_car_init(TIM_HandleTypeDef* left_motor, uint32_t left_channel,
                           TIM_HandleTypeDef* right_motor, uint32_t right_channel);

/**
  * @brief          设置电机方向
  * @param[in]      motor_index: 电机索引
  * @param[in]      dir: 电机方向
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_car_set_motor_direction(uint8_t motor_index, motor_dir_e dir);

/**
  * @brief          设置电机速度
  * @param[in]      motor_index: 电机索引
  * @param[in]      speed: 速度值(-100至100)，正值前进，负值后退
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_car_set_motor_speed(uint8_t motor_index, float speed);

/**
  * @brief          获取电机当前速度
  * @param[in]      motor_index: 电机索引
  * @retval         速度值(-100至100)
  */
extern float bsp_car_get_motor_speed(uint8_t motor_index);

/**
  * @brief          设置小车运动
  * @param[in]      forward_speed: 前进速度(-100至100)
  * @param[in]      turn_rate: 转向速率(-100至100)，正值右转，负值左转
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_car_set_motion(float forward_speed, float turn_rate);

/**
  * @brief          设置小车目标运动并启用平滑控制
  * @param[in]      target_forward: 目标前进速度(-100至100)
  * @param[in]      target_turn: 目标转向速率(-100至100)
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_car_set_target_motion(float target_forward, float target_turn);

/**
  * @brief          小车运动控制处理函数，应在循环中调用以实现平滑控制
  * @param[in]      none
  * @retval         none
  */
extern void bsp_car_motion_ctrl_update(void);

/**
  * @brief          遥控器控制小车
  * @param[in]      forward_ch: 前进通道值(-660至660)
  * @param[in]      turn_ch: 转向通道值(-660至660)
  * @retval         返回0表示成功，非0表示失败
  */
extern uint8_t bsp_car_control_by_rc(int16_t forward_ch, int16_t turn_ch);

/**
  * @brief          停止小车
  * @param[in]      none
  * @retval         none
  */
extern void bsp_car_stop(void);

/**
  * @brief          获取小车初始化状态
  * @retval         返回1表示已初始化，0表示未初始化
  */
extern uint8_t bsp_car_is_initialized(void);

#endif /* BSP_CAR_H */