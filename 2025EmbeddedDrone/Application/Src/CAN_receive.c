/**
  ******************************************************************************
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
  ******************************************************************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "user_lib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  can1_tx_message;
static uint8_t              can1_send_data[8];

static motor_measure_t motor_measure[2];


//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->u16_last_pos = (ptr)->u16_pos;                           \
        (ptr)->u16_pos = ((data)[0] << 8 | (data)[1]);                  \
        (ptr)->int16_vel = ((data)[2] << 8 | (data)[3]);                \
        (ptr)->int16_torq = ((data)[4] << 8 | (data)[5]);               \
        (ptr)->u8_temperate = (data)[6];                                \
    }


/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static bool_t is_init = 1;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_LEFT_M2006_RX_ID:
        {
            get_motor_measure(&motor_measure[0], rx_data);
//            detect_hook(TRIGGER_MOTOR_TOE);
            break;
        }
        case CAN_RIGHT_M2006_RX_ID:
        {
            get_motor_measure(&motor_measure[1], rx_data);
//            detect_hook(TRIGGER_MOTOR_TOE);
            break;
        }
        default:
        {
            break;
        }
    }
    
    if(is_init)is_init = 0;
}


/**
  * @brief          发送拨盘电机M2006控制电流
  * @param[in]      current: 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd(int16_t left_current, int16_t right_current)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = CAN_M2006_TX_ID;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = (left_current >> 8);
    can1_send_data[1] = left_current;
    can1_send_data[2] = (right_current >> 8);
    can1_send_data[3] = right_current;
    can1_send_data[4] = 0;
    can1_send_data[5] = 0;
    can1_send_data[6] = 0;
    can1_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}


/**
  * @brief          返回右电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_left_motor_measure_point(void)
{
    return &motor_measure[0];
}


/**
  * @brief          返回右电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_right_motor_measure_point(void)
{
    return &motor_measure[1];
}

