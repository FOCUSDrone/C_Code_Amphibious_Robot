/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       chassis_control.h
  * @brief      底盘控制层，处理运动学解算和差速控制
  * @note       作为car_task的底层模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Claude          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "struct_typedef.h"

/* 底盘控制参数 */
#define CHASSIS_SPEED_MAX      5000.0f  // 最大前进速度，单位RPM
#define CHASSIS_TURN_SPEED_MAX 4000.0f  // 最大转向速度，单位RPM

/**
  * @brief          底盘控制初始化
  * @param[in]      none
  * @retval         none
  */
extern void chassis_control_init(void);

/**
  * @brief          根据遥控器输入设置底盘控制指令
  * @param[in]      forward_back: 前后通道值，范围[-660, 660]
  * @param[in]      left_right: 左右通道值，范围[-660, 660]
  * @retval         none
  */
extern void chassis_set_control(int16_t forward_back, int16_t left_right);

/**
  * @brief          底盘控制计算，在周期任务中调用
  * @param[in]      none
  * @retval         none
  */
extern void chassis_control_calc(void);

/**
  * @brief          获取底盘模式
  * @param[in]      none
  * @retval         底盘模式 0:停止 1:正常
  */
extern uint8_t get_chassis_mode(void);

#endif


