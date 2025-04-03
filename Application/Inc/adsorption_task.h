/**
  ******************************************************************************
  * @file       adsorption_task.h
  * @brief      吸附线程头文件，通过遥控器控制风扇转速
  * @note       使用遥控器通道3和开关控制，避免与car_task冲突
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-25      Feiziben        1. 完成
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef ADSORPTION_TASK_H
#define ADSORPTION_TASK_H

#include "struct_typedef.h"

/* 任务控制周期，与风扇控制周期匹配 */
#define ADSORPTION_TASK_TIME       10

/* 使用的遥控器通道和开关 */
#define FAN_CONTROL_CHANNEL        2    /* 右摇杆上下方向(通道3)控制风扇转速 */
#define FAN_CONTROL_SWITCH         0    /* 使用第2个开关控制风扇模式 */

/* 风扇控制模式枚举 */
typedef enum 
{
    FAN_MODE_OFF = 0,        /* 风扇关闭模式 */
    FAN_MODE_MANUAL,         /* 手动控制模式 */
    FAN_MODE_AUTO            /* 自动控制模式（预留） */
} fan_control_mode_e;

/* 风扇控制状态结构体 */
typedef struct 
{
    fan_control_mode_e mode;    /* 当前控制模式 */
    uint16_t target_rpm;        /* 目标转速 */
    uint16_t current_rpm;       /* 当前转速 */
    int16_t remote_value;       /* 遥控器输入值 */
    uint8_t is_active;          /* 风扇是否激活 */
} fan_control_t;

/**
  * @brief          吸附任务，在RTOS中注册为线程
  * @param[in]      argument: 线程参数
  * @retval         none
  */
void adsorption_task(void const * argument);

/**
  * @brief          获取风扇控制状态指针
  * @param[in]      none
  * @retval         风扇控制状态结构体指针
  */
const fan_control_t *get_fan_control_point(void);

#endif /* ADSORPTION_TASK_H */
