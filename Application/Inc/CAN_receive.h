/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       can_receive.h
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

/* CANͨ�����ID���� */
#define CAN_2006_M1_ID          0x201  // ����ID
#define CAN_2006_M2_ID          0x202  // �ҵ��ID
#define CAN_CHASSIS_ALL_ID      0x200  // ���̿���ID

/* ������ݽṹ�� */
typedef struct {
    uint16_t ecd;                  // ������ֵ
    int16_t speed_rpm;             // ת��
    int16_t given_current;         // ��������
    uint8_t temperate;             // �¶�
    uint16_t last_ecd;             // �ϴα�����ֵ
} motor_measure_t;

/**
  * @brief          ���͵�����Ƶ���(0x201,0x202��Ӧ�����ҵ��)
  * @param[in]      motor1: �������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor2: �ҵ�����Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2);

/**
  * @brief          ������������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor1_measure_point(void);

/**
  * @brief          �����ҵ������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor2_measure_point(void);

#endif

