/**
  ******************************************************************************
  * @file       car_task.h
  * @brief      С���������񣬻���ң����ʵ�ֲ���С������
  * @note       ʹ��FreeRTOSʵ�֣�����bsp_car����
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_car.h"

/* ����������� */
#define CAR_TASK_PRIO                20        // �������ȼ�
#define CAR_STK_SIZE                 128       // ����ջ��С
#define CAR_TASK_INIT_TIME           100       // ��ʼ����ʱ(ms)
#define CAR_TASK_PERIOD              10        // ��������(ms)

/* ң����ͨ������ */
#define CAR_FORWARD_CHANNEL          1         // ǰ������ͨ��(��ҡ��Y��)
#define CAR_TURN_CHANNEL             2         // ת��ͨ��(��ҡ��X��)
#define CAR_MODE_SWITCH_CHANNEL      5         // ģʽ�л�ͨ��
#define CAR_EMERGENCY_CHANNEL        6         // ����ֹͣͨ��

/* ���Ʋ������� */
#define CAR_FORWARD_SCALE            1.0f      // ǰ���ٶ����ű���
#define CAR_TURN_SCALE               0.8f      // ת���ٶ����ű���
#define CAR_CONTROL_DEADBAND         20        // ��������ֵ
#define CAR_STICK_MIDDLE_OFFSET      0         // ҡ����ֵƫ������

/* �ٶȽ������ */
#define CAR_SPEED_RAMP_NORMAL        8.0f      // ����Ӽ���б��(%/����)
#define CAR_SPEED_RAMP_SLOW          3.0f      // ���ټӼ���б��(%/����)
#define CAR_SPEED_RAMP_FAST          15.0f     // ���ټӼ���б��(%/����)

/* ��ȫ���� */
#define CAR_RC_LOST_TIME             1000      // ң������ʧ�ж�ʱ��(ms)
#define CAR_TIMEOUT_CHECK_PERIOD     100       // ��ʱ�������(ms)

/* С������ģʽö�� */
typedef enum {
    CAR_MODE_STOP = 0,           // ֹͣģʽ
    CAR_MODE_NORMAL,             // ��ͨģʽ
    CAR_MODE_SPORT,              // �˶�ģʽ(��������)
    CAR_MODE_ECO,                // ����ģʽ(����ʡ��)
    CAR_MODE_AUTO,               // �Զ�ģʽ
    CAR_MODE_EMERGENCY           // ����ģʽ
} car_mode_e;

/* С������״̬ö�� */
typedef enum {
    CAR_STATE_STOP = 0,          // ֹͣ״̬
    CAR_STATE_FORWARD,           // ǰ��״̬
    CAR_STATE_BACKWARD,          // ����״̬
    CAR_STATE_TURNING,           // ת��״̬
    CAR_STATE_ERROR              // ����״̬
} car_state_e;

/* С��������Ϣ�ṹ�� */
typedef struct {
    car_mode_e      mode;          // ��ǰ����ģʽ
    car_state_e     state;         // ��ǰ����״̬
    float           forward_speed; // ��ǰǰ���ٶ�(-100��100)
    float           turn_rate;     // ��ǰת����(-100��100)
    float           left_speed;    // �����ٶ�(-100��100)
    float           right_speed;   // �ҵ���ٶ�(-100��100)
    uint32_t        runtime;       // ����ʱ��(ms)
    uint8_t         error_code;    // �������
    uint8_t         rc_connected;  // ң��������״̬
} car_info_t;

/* �������� */
/**
  * @brief          С���������񣬴���ң�������벢����С���˶�
  * @param[in]      argument: �������
  * @retval         void
  */
extern void car_task(void const * argument);

/**
  * @brief          С������ϵͳ��ʼ��
  * @param[in]      left_tim: ������ʱ�����
  * @param[in]      left_channel: ����PWMͨ��
  * @param[in]      right_tim: �ҵ����ʱ�����
  * @param[in]      right_channel: �ҵ��PWMͨ��
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t car_system_init(TIM_HandleTypeDef* left_tim, uint32_t left_channel,
                               TIM_HandleTypeDef* right_tim, uint32_t right_channel);

/**
  * @brief          ����С������ģʽ
  * @param[in]      mode: ����ģʽ
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t car_set_mode(car_mode_e mode);

/**
  * @brief          �ֶ�����С���˶�
  * @param[in]      forward_speed: ǰ���ٶ�(-100��100)
  * @param[in]      turn_rate: ת������(-100��100)
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t car_manual_control(float forward_speed, float turn_rate);

/**
  * @brief          ��ȡС��������Ϣ
  * @param[out]     info: ��Ϣ�ṹ��ָ��
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t car_get_info(car_info_t* info);

/**
  * @brief          ����ֹͣС��
  * @param[in]      none
  * @retval         none
  */
extern void car_emergency_stop(void);

#endif /* CAR_TASK_H */