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
        
        default:
        {
            break;
        }
    }
    
    if(is_init)is_init = 0;
}
