/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       can_receive.h
  * @brief      CAN通信接收处理，用于接收电机数据并控制电机
  * @note       专为2006电机差速小车应用优化
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Claude          1. 重构优化
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

/* CAN通信相关ID定义 */
#define CAN_2006_M1_ID          0x201  // 左电机ID
#define CAN_2006_M2_ID          0x202  // 右电机ID
#define CAN_CHASSIS_ALL_ID      0x200  // 底盘控制ID

/* 电机数据结构体 */
typedef struct {
    uint16_t ecd;                  // 编码器值
    int16_t speed_rpm;             // 转速
    int16_t given_current;         // 给定电流
    uint8_t temperate;             // 温度
    uint16_t last_ecd;             // 上次编码器值
} motor_measure_t;

/**
  * @brief          发送电机控制电流(0x201,0x202对应于左右电机)
  * @param[in]      motor1: 左电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: 右电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2);

/**
  * @brief          返回左电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor1_measure_point(void);

/**
  * @brief          返回右电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor2_measure_point(void);

#endif

