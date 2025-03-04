/**
  ******************************************************************************
  * @file       car_task.h
  * @brief      小车控制任务，基于遥控器实现差速小车控制
  * @note       使用FreeRTOS实现，依赖bsp_car驱动
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_car.h"

/* 任务参数配置 */
#define CAR_TASK_PRIO                20        // 任务优先级
#define CAR_STK_SIZE                 128       // 任务栈大小
#define CAR_TASK_INIT_TIME           100       // 初始化延时(ms)
#define CAR_TASK_PERIOD              10        // 任务周期(ms)

/* 遥控器通道配置 */
#define CAR_FORWARD_CHANNEL          1         // 前进后退通道(左摇杆Y轴)
#define CAR_TURN_CHANNEL             2         // 转向通道(右摇杆X轴)
#define CAR_MODE_SWITCH_CHANNEL      5         // 模式切换通道
#define CAR_EMERGENCY_CHANNEL        6         // 紧急停止通道

/* 控制参数配置 */
#define CAR_FORWARD_SCALE            1.0f      // 前进速度缩放比例
#define CAR_TURN_SCALE               0.8f      // 转向速度缩放比例
#define CAR_CONTROL_DEADBAND         20        // 控制死区值
#define CAR_STICK_MIDDLE_OFFSET      0         // 摇杆中值偏移修正

/* 速度渐变参数 */
#define CAR_SPEED_RAMP_NORMAL        8.0f      // 常规加减速斜率(%/周期)
#define CAR_SPEED_RAMP_SLOW          3.0f      // 慢速加减速斜率(%/周期)
#define CAR_SPEED_RAMP_FAST          15.0f     // 快速加减速斜率(%/周期)

/* 安全参数 */
#define CAR_RC_LOST_TIME             1000      // 遥控器丢失判定时间(ms)
#define CAR_TIMEOUT_CHECK_PERIOD     100       // 超时检查周期(ms)

/* 小车控制模式枚举 */
typedef enum {
    CAR_MODE_STOP = 0,           // 停止模式
    CAR_MODE_NORMAL,             // 普通模式
    CAR_MODE_SPORT,              // 运动模式(高灵敏度)
    CAR_MODE_ECO,                // 经济模式(低速省电)
    CAR_MODE_AUTO,               // 自动模式
    CAR_MODE_EMERGENCY           // 紧急模式
} car_mode_e;

/* 小车运行状态枚举 */
typedef enum {
    CAR_STATE_STOP = 0,          // 停止状态
    CAR_STATE_FORWARD,           // 前进状态
    CAR_STATE_BACKWARD,          // 后退状态
    CAR_STATE_TURNING,           // 转向状态
    CAR_STATE_ERROR              // 错误状态
} car_state_e;

/* 小车控制信息结构体 */
typedef struct {
    car_mode_e      mode;          // 当前控制模式
    car_state_e     state;         // 当前运行状态
    float           forward_speed; // 当前前进速度(-100至100)
    float           turn_rate;     // 当前转向率(-100至100)
    float           left_speed;    // 左电机速度(-100至100)
    float           right_speed;   // 右电机速度(-100至100)
    uint32_t        runtime;       // 运行时间(ms)
    uint8_t         error_code;    // 错误代码
    uint8_t         rc_connected;  // 遥控器连接状态
} car_info_t;

/* 函数声明 */
/**
  * @brief          小车控制任务，处理遥控器输入并控制小车运动
  * @param[in]      argument: 任务参数
  * @retval         void
  */
extern void car_task(void const * argument);

/**
  * @brief          小车控制系统初始化
  * @param[in]      left_tim: 左电机定时器句柄
  * @param[in]      left_channel: 左电机PWM通道
  * @param[in]      right_tim: 右电机定时器句柄
  * @param[in]      right_channel: 右电机PWM通道
  * @retval         0成功，非0失败
  */
extern uint8_t car_system_init(TIM_HandleTypeDef* left_tim, uint32_t left_channel,
                               TIM_HandleTypeDef* right_tim, uint32_t right_channel);

/**
  * @brief          设置小车控制模式
  * @param[in]      mode: 控制模式
  * @retval         0成功，非0失败
  */
extern uint8_t car_set_mode(car_mode_e mode);

/**
  * @brief          手动控制小车运动
  * @param[in]      forward_speed: 前进速度(-100至100)
  * @param[in]      turn_rate: 转向速率(-100至100)
  * @retval         0成功，非0失败
  */
extern uint8_t car_manual_control(float forward_speed, float turn_rate);

/**
  * @brief          获取小车控制信息
  * @param[out]     info: 信息结构体指针
  * @retval         0成功，非0失败
  */
extern uint8_t car_get_info(car_info_t* info);

/**
  * @brief          紧急停止小车
  * @param[in]      none
  * @retval         none
  */
extern void car_emergency_stop(void);

#endif /* CAR_TASK_H */