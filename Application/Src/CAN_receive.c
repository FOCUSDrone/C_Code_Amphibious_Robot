/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       can_receive.c
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

#include "can_receive.h"
#include "main.h"

/* 外部变量声明 */
extern CAN_HandleTypeDef hcan1;

/* 电机数据读取宏定义 */
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/* 电机数据存储 */
static motor_measure_t motor_chassis[2]; // 只存储2个电机数据

/* CAN发送相关变量 */
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
  * @brief          CAN接收中断回调函数，接收电机数据
  * @param[in]      hcan: CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* 获取接收到的数据 */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* 根据ID处理不同电机数据 */
    switch (rx_header.StdId)
    {
        case CAN_2006_M1_ID: // 左电机数据处理
            get_motor_measure(&motor_chassis[0], rx_data);
            break;
            
        case CAN_2006_M2_ID: // 右电机数据处理
            get_motor_measure(&motor_chassis[1], rx_data);
            break;
            
        default:
            break;
    }
}

/**
  * @brief          发送电机控制电流(0x201,0x202对应于左右电机)
  * @param[in]      motor1: 左电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: 右电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    
    /* 设置标准帧ID */
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    
    /* 组装数据 */
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    
    /* 填充剩余未使用字节为0 */
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    /* 发送CAN消息 */
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          返回左电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor1_measure_point(void)
{
    return &motor_chassis[0];
}

/**
  * @brief          返回右电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor2_measure_point(void)
{
    return &motor_chassis[1];
}