/**
  **********************************************************************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-2-7        tanjiong        1. done
  *  
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **********************************************************************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

typedef enum
{
	CAN_M2006_TX_ID = 0x200,
    CAN_LEFT_M2006_RX_ID = 0x201,
    CAN_RIGHT_M2006_RX_ID = 0x202
} can_msg_id_e;

typedef struct
{
    uint16_t u16_pos;
    int16_t int16_vel;
    int16_t int16_torq;
    uint16_t u16_last_pos;
	uint8_t u8_temperate;
} motor_measure_t;


/**
  * @brief          发送拨盘电机M2006控制电流
  * @param[in]      current: 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd(int16_t left_current, int16_t right_current);

/**
  * @brief          返回右电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_left_motor_measure_point(void);

/**
  * @brief          返回右电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_right_motor_measure_point(void);

#endif
