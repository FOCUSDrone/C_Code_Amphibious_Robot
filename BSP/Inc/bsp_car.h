/**
  ******************************************************************************
  * @file       bsp_car.h
  * @brief      С���������BSP�㣬֧�ֲ���ת��
  * @note       ����STM32 HAL�⣬ʹ��PWM���Ƶ��
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. ������
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
  
#ifndef BSP_CAR_H
#define BSP_CAR_H

#include "main.h"

/* ������������� */
#define CAR_MOTOR_NUM            2        // �������
#define CAR_LEFT_MOTOR_INDEX     0        // ��������
#define CAR_RIGHT_MOTOR_INDEX    1        // �ҵ������

/* PWM���� */
#define CAR_PWM_TIM_FREQ         1000000  // ��ʱ��Ƶ�� 1MHz
#define CAR_PWM_FREQ             50       // PWMƵ�� 50Hz (��׼���)
#define CAR_PWM_PERIOD           (CAR_PWM_TIM_FREQ / CAR_PWM_FREQ) // PWM����

/* ���PWM����(��׼���Ϊ1000-2000us) */
#define CAR_PWM_NEUTRAL          1500     // �м�ֵ�����ֹͣ��
#define CAR_PWM_FORWARD_MAX      2000     // ���ǰ��ֵ
#define CAR_PWM_BACKWARD_MAX     1000     // ������ֵ
#define CAR_PWM_DEADBAND         50       // ������Χ

/* ת����Ʋ��� */
#define CAR_TURN_RATE            0.5f     // ת��������(0-1)
#define CAR_SPEED_RAMP_RATE      10       // �ٶȽ�����

/* ң�������Ʋ��� */
#define CAR_RC_DEADBAND          5        // ң��������
#define CAR_RC_MAX_VALUE         660      // ң�������ֵ
#define CAR_RC_MIN_VALUE         -660     // ң������Сֵ

/* ��������� */
typedef enum {
    MOTOR_DIR_NORMAL = 0,        // ��������
    MOTOR_DIR_REVERSE = 1,       // ����
} motor_dir_e;

/* ���PWMͨ������ */
typedef struct {
    TIM_HandleTypeDef* htim;     // ��ʱ�����ָ��
    uint32_t           channel;  // ��ʱ��ͨ��
    motor_dir_e        dir;      // �������
    uint8_t            is_active; // �Ƿ񼤻�
} motor_pwm_t;

/* С������ģʽö�� */
typedef enum {
    CAR_CTRL_MANUAL = 0,         // �ֶ�����ģʽ
    CAR_CTRL_RC,                 // ң��������ģʽ
    CAR_CTRL_AUTO,               // �Զ�����ģʽ
} car_ctrl_mode_e;

/* �������� */
/**
  * @brief          С��BSP��ʼ��
  * @param[in]      left_motor: ������ʱ�����
  * @param[in]      left_channel: ����PWMͨ��
  * @param[in]      right_motor: �ҵ����ʱ�����
  * @param[in]      right_channel: �ҵ��PWMͨ��
  * @retval         ����0��ʾ��ʼ���ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_init(TIM_HandleTypeDef* left_motor, uint32_t left_channel,
                           TIM_HandleTypeDef* right_motor, uint32_t right_channel);

/**
  * @brief          ���õ������
  * @param[in]      motor_index: �������
  * @param[in]      dir: �������
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_set_motor_direction(uint8_t motor_index, motor_dir_e dir);

/**
  * @brief          ���õ���ٶ�
  * @param[in]      motor_index: �������
  * @param[in]      speed: �ٶ�ֵ(-100��100)����ֵǰ������ֵ����
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_set_motor_speed(uint8_t motor_index, float speed);

/**
  * @brief          ��ȡ�����ǰ�ٶ�
  * @param[in]      motor_index: �������
  * @retval         �ٶ�ֵ(-100��100)
  */
extern float bsp_car_get_motor_speed(uint8_t motor_index);

/**
  * @brief          ����С���˶�
  * @param[in]      forward_speed: ǰ���ٶ�(-100��100)
  * @param[in]      turn_rate: ת������(-100��100)����ֵ��ת����ֵ��ת
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_set_motion(float forward_speed, float turn_rate);

/**
  * @brief          ����С��Ŀ���˶�������ƽ������
  * @param[in]      target_forward: Ŀ��ǰ���ٶ�(-100��100)
  * @param[in]      target_turn: Ŀ��ת������(-100��100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_set_target_motion(float target_forward, float target_turn);

/**
  * @brief          С���˶����ƴ�������Ӧ��ѭ���е�����ʵ��ƽ������
  * @param[in]      none
  * @retval         none
  */
extern void bsp_car_motion_ctrl_update(void);

/**
  * @brief          ң��������С��
  * @param[in]      forward_ch: ǰ��ͨ��ֵ(-660��660)
  * @param[in]      turn_ch: ת��ͨ��ֵ(-660��660)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_car_control_by_rc(int16_t forward_ch, int16_t turn_ch);

/**
  * @brief          ֹͣС��
  * @param[in]      none
  * @retval         none
  */
extern void bsp_car_stop(void);

/**
  * @brief          ��ȡС����ʼ��״̬
  * @retval         ����1��ʾ�ѳ�ʼ����0��ʾδ��ʼ��
  */
extern uint8_t bsp_car_is_initialized(void);

#endif /* BSP_CAR_H */