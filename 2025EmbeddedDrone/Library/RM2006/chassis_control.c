/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       chassis_control.c
  * @brief      底盘控制层，处理运动学解算和差速控制
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

#include "chassis_control.h"
#include "motor_control.h"
#include "remote_receive.h"
#include <math.h>

/* 底盘控制结构体 */
typedef struct {
    /* 控制数据 */
    fp32 forward_speed;      // 前进速度
    fp32 turn_speed;         // 转向速度
    
    /* 控制模式 */
    uint8_t mode;            // 0:停止 1:正常
} chassis_control_t;

/* 全局变量 */
static chassis_control_t chassis;  // 底盘控制数据

/**
  * @brief          底盘控制初始化
  * @param[in]      none
  * @retval         none
  */
void chassis_control_init(void)
{
    // 初始化控制参数
    chassis.forward_speed = 0.0f;
    chassis.turn_speed = 0.0f;
    chassis.mode = 0;  // 默认停止模式
}

/**
  * @brief          根据遥控器输入设置底盘控制指令
  * @param[in]      forward_back: 前后通道值，范围[-660, 660]
  * @param[in]      left_right: 左右通道值，范围[-660, 660]
  * @retval         none
  */
void chassis_set_control(int16_t forward_back, int16_t left_right)
{
    // 遥控器数据死区处理
    int16_t forward_back_channel = 0;
    int16_t left_right_channel = 0;
    
    remote_deadband_limit(forward_back, forward_back_channel, REMOTE_DEADBAND);
    remote_deadband_limit(left_right, left_right_channel, REMOTE_DEADBAND);
    
    // 检查是否有控制输入，决定模式
    if (abs(forward_back_channel) <= REMOTE_DEADBAND && abs(left_right_channel) <= REMOTE_DEADBAND)
    {
        // 无控制输入，进入停止模式
        chassis.mode = 0;
        chassis.forward_speed = 0.0f;
        chassis.turn_speed = 0.0f;
    }
    else
    {
        // 有控制输入，进入正常模式
        chassis.mode = 1;
        
        // 计算前进速度和转向速度，映射到最大速度范围
        chassis.forward_speed = forward_back_channel / 660.0f * CHASSIS_SPEED_MAX;
        chassis.turn_speed = left_right_channel / 660.0f * CHASSIS_TURN_SPEED_MAX;
    }
}

/**
  * @brief          底盘控制计算，在周期任务中调用
  * @param[in]      none
  * @retval         none
  */
void chassis_control_calc(void)
{
    // 差速计算：左轮速度 = 前进速度 - 转向速度，右轮速度 = 前进速度 + 转向速度
    fp32 left_wheel_speed = chassis.forward_speed - chassis.turn_speed;
    fp32 right_wheel_speed = chassis.forward_speed + chassis.turn_speed;
    
    // 设置电机速度
    motor_speed_set(left_wheel_speed, right_wheel_speed);
}

/**
  * @brief          获取底盘模式
  * @param[in]      none
  * @retval         底盘模式 0:停止 1:正常
  */
uint8_t get_chassis_mode(void)
{
    return chassis.mode;
}

