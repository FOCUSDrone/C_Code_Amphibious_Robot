/**
 * @file bsp_fan.h
 * @brief DJI C板风扇驱动BSP（仅闭环控制）
 * @details 使用TIM8_CH1输出PWM控制风扇，GPIO(PC6)输入测量FG反馈信号
 * @author Feiziben
 * @date 2025.03.20
 */
#ifndef BSP_FAN_H
#define BSP_FAN_H

#include "main.h"
#include "tim.h"
#include "gpio.h"

/* 风扇控制常量 */
#define FAN_PWM_FREQ           1000    // PWM频率1kHz
#define FAN_MAX_DUTY           65535   // 最大占空比值(改为与TIM8周期值匹配)
#define PID_OUTPUT_MAX         65535   // 最大输出值
#define FAN_START_DUTY         600     // 启动占空比6%
#define FAN_STOP_DUTY          400     // 停止占空比4%
#define FAN_FG_FILTER_SIZE     5       // FG信号滤波窗口大小
#define FAN_FG_TIMEOUT_MS      2000    // FG信号超时时间(ms)
#define FAN_CONTROL_PERIOD_MS  10      // 控制周期(ms)
#define FAN_RPM_MAX            64500   // 最大转速约64500RPM

/* PID结构体 */
typedef struct {
    float kp;          // 比例系数
    float ki;          // 积分系数
    float kd;          // 微分系数
    float error;       // 当前误差
    float error_sum;   // 误差累加
    float error_last;  // 上次误差
    float output;      // PID输出
    float output_max;  // 输出限幅
    float output_min;  // 输出下限
} fan_pid_t;

/**
 * @brief 风扇初始化
 * @param 无
 * @return 无
 */
void fan_init(void);

/**
 * @brief 设置风扇目标转速
 * @param[in] target_rpm 目标转速(0-FAN_RPM_MAX)
 * @return 无
 */
void fan_set_rpm(uint16_t target_rpm);

/**
 * @brief 停止风扇
 * @param 无
 * @return 无
 */
void fan_stop(void);

/**
 * @brief 获取风扇转速
 * @param 无
 * @return 风扇转速(RPM)
 */
uint16_t fan_get_rpm(void);

/**
 * @brief 获取风扇目标转速
 * @param 无
 * @return 目标转速(RPM)
 */
uint16_t fan_get_target_rpm(void);

/**
 * @brief 风扇FG信号GPIO外部中断回调
 * @note 需在HAL_GPIO_EXTI_Callback中调用
 * @param[in] GPIO_Pin 触发中断的GPIO引脚
 * @return 无
 */
void fan_fg_exti_callback(uint16_t GPIO_Pin);

/**
 * @brief 风扇控制周期处理
 * @note 需在主循环或定时任务中每10ms调用一次
 * @param 无
 * @return 无
 */
void fan_process(void);

#endif /* BSP_FAN_H */


