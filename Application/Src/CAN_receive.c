/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       can_receive.c
  * @brief      CANͨ�Ž��մ������ڽ��յ�����ݲ����Ƶ��
  * @note       רΪ2006�������С��Ӧ���Ż�
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Claude          1. �ع��Ż�
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#include "can_receive.h"
#include "main.h"

/* �ⲿ�������� */
extern CAN_HandleTypeDef hcan1;

/* ������ݶ�ȡ�궨�� */
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/* ������ݴ洢 */
static motor_measure_t motor_chassis[2]; // ֻ�洢2���������

/* CAN������ر��� */
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
  * @brief          CAN�����жϻص����������յ������
  * @param[in]      hcan: CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* ��ȡ���յ������� */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* ����ID����ͬ������� */
    switch (rx_header.StdId)
    {
        case CAN_2006_M1_ID: // �������ݴ���
            get_motor_measure(&motor_chassis[0], rx_data);
            break;
            
        case CAN_2006_M2_ID: // �ҵ�����ݴ���
            get_motor_measure(&motor_chassis[1], rx_data);
            break;
            
        default:
            break;
    }
}

/**
  * @brief          ���͵�����Ƶ���(0x201,0x202��Ӧ�����ҵ��)
  * @param[in]      motor1: �������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor2: �ҵ�����Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    
    /* ���ñ�׼֡ID */
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    
    /* ��װ���� */
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    
    /* ���ʣ��δʹ���ֽ�Ϊ0 */
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    /* ����CAN��Ϣ */
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          ������������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor1_measure_point(void)
{
    return &motor_chassis[0];
}

/**
  * @brief          �����ҵ������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor2_measure_point(void)
{
    return &motor_chassis[1];
}