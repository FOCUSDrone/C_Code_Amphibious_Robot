/**
  ******************************************************************************
  * @file       Adhesion_task.h
  * @brief      ����ϵͳ�������񣬻��ڷ������ʵ�ֿɿ���������
  * @note       ʹ��FreeRTOSʵ�֣�����bsp_fan����
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   YourName    1. ���
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef ADHESION_TASK_H
#define ADHESION_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_fan.h"

/* ����������� */
#define ADHESION_TASK_PRIO             15       // �������ȼ�
#define ADHESION_STK_SIZE              128      // ����ջ��С
#define ADHESION_TASK_INIT_TIME        100      // ��ʼ����ʱ(ms)
#define ADHESION_TASK_PERIOD           10       // ��������(ms)

/* ���Ʋ������� */
#define ADHESION_MODE_CHANNEL          5        // ����ģʽ�л�ͨ��
#define ADHESION_POWER_CHANNEL         2        // �������ʿ���ͨ��
#define ADHESION_SAFE_TEMP             70       // ��ȫ�¶���ֵ(��C)
#define ADHESION_CONTROL_INTERVAL      50       // ���Ƹ��¼��(ms)

/* ����ģʽ���� */
typedef enum {
    ADHESION_MODE_OFF = 0,             // �ر�ģʽ
    ADHESION_MODE_MANUAL,              // �ֶ�����ģʽ
    ADHESION_MODE_AUTO,                // �Զ�����ģʽ
    ADHESION_MODE_SMART                // ���ܿ���ģʽ
} adhesion_mode_e;

/* ����״̬���� */
typedef enum {
    ADHESION_STATE_OFF = 0,            // �ر�״̬
    ADHESION_STATE_STARTING,           // ������
    ADHESION_STATE_RUNNING,            // ������
    ADHESION_STATE_STOPPING,           // ֹͣ��
    ADHESION_STATE_ERROR               // ����״̬
} adhesion_state_e;

/* �����������ݽṹ */
typedef struct {
    adhesion_mode_e     mode;          // ��ǰ����ģʽ
    adhesion_state_e    state;         // ��ǰ״̬
    float               power;         // ��ǰ����(%)
    float               target_power;  // Ŀ�깦��(%)
    uint32_t            runtime;       // ����ʱ��(ms)
    uint8_t             fan_index;     // �������
    uint8_t             error_code;    // �������
    const int16_t*      rc_data;       // ң��������ָ��
} adhesion_info_t;

/* �������� */
/**
  * @brief          ����ϵͳ����ͨ��ң�������Ʒ��ʵ����������
  * @param[in]      argument: �������ָ��
  * @retval         void
  */
extern void Adhesion_task(void const * argument);

/**
  * @brief          ����ϵͳ��ʼ��
  * @param[in]      fan_tim: ���PWM��ʱ�����
  * @param[in]      fan_channel: ���PWMͨ��
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t adhesion_system_init(TIM_HandleTypeDef* fan_tim, uint32_t fan_channel);

/**
  * @brief          ��������ϵͳģʽ
  * @param[in]      mode: ����ģʽ
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t adhesion_set_mode(adhesion_mode_e mode);

/**
  * @brief          ������������
  * @param[in]      power: ���ʰٷֱ�(0-100)
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t adhesion_set_power(float power);

/**
  * @brief          ��ȡ����ϵͳ��Ϣ
  * @param[out]     info: ����ϵͳ��Ϣ�ṹָ��
  * @retval         0�ɹ�����0ʧ��
  */
extern uint8_t adhesion_get_info(adhesion_info_t* info);

/**
  * @brief          ����ֹͣ����ϵͳ
  * @param[in]      none
  * @retval         none
  */
extern void adhesion_emergency_stop(void);

#endif /* ADHESION_TASK_H */

