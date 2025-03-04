/**
  ******************************************************************************
  * @file       bsp_fan.h
  * @brief      ���PWM����BSP��
  * @note       ����STM32 HAL�⣬֧�ֶ������ƺ�ƽ��ת�ٱ仯
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
  
#ifndef BSP_FAN_H
#define BSP_FAN_H

#include "main.h"

/* ���������ͨ������ */
#define FAN_NUM                 1        // �������
#define FAN_MAIN_INDEX          0        // ���������

/* PWM���� */
#define FAN_PWM_TIM_FREQ        1000000  // ��ʱ��Ƶ�� 1MHz
#define FAN_PWM_FREQ            50       // PWMƵ�� 50Hz
#define FAN_PWM_RESOLUTION      10000    // PWM����(0-10000)
#define FAN_PWM_PERIOD          (FAN_PWM_TIM_FREQ / FAN_PWM_FREQ) // PWM����

/* ת�ٷ�Χ�Ϳ��Ʋ��� */
#define FAN_SPEED_MAX           100      // ���ת�ٰٷֱ�
#define FAN_SPEED_MIN           0        // ��Сת�ٰٷֱ�
#define FAN_SPEED_DEFAULT       0        // Ĭ��ת�ٰٷֱ�
#define FAN_RAMP_RATE           2        // ÿ�ο���ת�ٱ仯��(%)

/* ң�������Ʋ��� */
#define FAN_RC_DEADBAND         5        // ң��������
#define FAN_RC_MIN_VALUE        -660     // ң������Сֵ
#define FAN_RC_MAX_VALUE        660      // ң�������ֵ

/* ���PWMͨ������ */
typedef struct {
    TIM_HandleTypeDef* htim;     // ��ʱ�����ָ��
    uint32_t           channel;  // ��ʱ��ͨ��
    uint8_t            is_active; // �Ƿ񼤻�
} fan_pwm_t;

/* �������ģʽö�� */
typedef enum {
    FAN_CTRL_MANUAL = 0,        // �ֶ�����ģʽ
    FAN_CTRL_RC,                // ң��������ģʽ
    FAN_CTRL_AUTO,              // �Զ�����ģʽ
} fan_ctrl_mode_e;

/* �������� */
/**
  * @brief          ���BSP��ʼ��
  * @param[in]      htim: ���PWM��ʱ�����ָ��
  * @param[in]      channel: ���PWMͨ��
  * @retval         ����0��ʾ��ʼ���ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_fan_init(TIM_HandleTypeDef* htim, uint32_t channel);

/**
  * @brief          ���÷��ת�ٰٷֱ�
  * @param[in]      fan_index: �������
  * @param[in]      percent: ת�ٰٷֱ�(0-100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_fan_set_speed(uint8_t fan_index, float percent);

/**
  * @brief          ��ȡ�����ǰת�ٰٷֱ�
  * @param[in]      fan_index: �������
  * @retval         ���ת�ٰٷֱ�(0-100)
  */
extern float bsp_fan_get_speed(uint8_t fan_index);

/**
  * @brief          ���÷��Ŀ��ת�ٲ�����ƽ������
  * @param[in]      fan_index: �������
  * @param[in]      target_percent: Ŀ��ת�ٰٷֱ�(0-100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_fan_set_target_speed(uint8_t fan_index, float target_percent);

/**
  * @brief          ����ٶȿ��ƴ�������Ӧ��ѭ���е�����ʵ��ƽ������
  * @param[in]      none
  * @retval         none
  */
extern void bsp_fan_speed_ctrl_update(void);

/**
  * @brief          ң�������Ʒ��
  * @param[in]      fan_index: �������
  * @param[in]      rc_value: ң����ֵ(-660��660)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
extern uint8_t bsp_fan_control_by_rc(uint8_t fan_index, int16_t rc_value);

/**
  * @brief          ֹͣ���з��
  * @param[in]      none
  * @retval         none
  */
extern void bsp_fan_stop_all(void);

/**
  * @brief          ��ȡ�����ʼ��״̬
  * @param[in]      fan_index: �������
  * @retval         ����1��ʾ�ѳ�ʼ����0��ʾδ��ʼ��
  */
extern uint8_t bsp_fan_is_initialized(uint8_t fan_index);

#endif /* BSP_FAN_H */