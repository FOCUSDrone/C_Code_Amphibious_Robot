/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       motor_control.h
  * @brief      电机控制层，处理电机速度控制及PID调节
  * @note       作为car_task的底层模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Feiziben          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "struct_typedef.h"

/* 电机参数配置 */
#define MOTOR_SPEED_MAX        5000    // 最大速度，单位RPM，适合2006电机
#define MOTOR_CURRENT_MAX      10000   // 最大电流限制，适用于RM2006
#define MOTOR_GEAR_RATIO       9       // 电机传动比 9:1

/* 电机配置，2006电机 */
#define LEFT_MOTOR_ID          0x201   // 左电机ID: 0x201
#define RIGHT_MOTOR_ID         0x202   // 右电机ID: 0x202

/* 电机PID参数 - 针对RM2006电机优化 */
#define MOTOR_SPEED_KP        12.0f
#define MOTOR_SPEED_KI        0.5f
#define MOTOR_SPEED_KD        0.0f

/**
  * @brief          电机控制初始化
  * @param[in]      none
  * @retval         none
  */
extern void motor_control_init(void);

/**
  * @brief          设置左右电机速度
  * @param[in]      left_speed: 左电机目标速度，范围[-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]
  * @param[in]      right_speed: 右电机目标速度，范围[-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]
  * @retval         none
  */
extern void motor_speed_set(fp32 left_speed, fp32 right_speed);

/**
  * @brief          电机控制计算与执行，在周期任务中调用
  * @param[in]      none
  * @retval         none
  */
extern void motor_control_calc(void);

#endif

