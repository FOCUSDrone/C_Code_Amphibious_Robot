/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       car_task.c
  * @brief      ����С����������ͨ��ң������������RM2006���ʵ�ֲ���ת��
  * @note       ��Ϊ����RTOS�߳�ʵ��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Feiziben          1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#include "car_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "remote_receive.h"
#include "motor_control.h"
#include "chassis_control.h"

/* ������ */
static TaskHandle_t car_task_handle;

/* ������ز��� */
#define CAR_TASK_PRIO        17      // �������ȼ�
#define CAR_STK_SIZE         512     // �����ջ��С

/**
  * @brief          С��������RTOS��ע��Ϊ�߳�
  * @param[in]      pvParameters: �̲߳���
  * @retval         none
  */
void car_task(void *pvParameters)
{
    // �ȴ����������ʼ�����
    vTaskDelay(CAR_TASK_INIT_TIME);
    
    // ��ʼ��CAN�����˲�����ȷ��ͨ������
    can_filter_init();
    
    // ��ʼ���������ģ��
    motor_control_init();
    
    // ��ʼ�����̿���ģ��
    chassis_control_init();
    
    // ��ʼ��ң�������չ���
    remote_receive_init();
    
    while (1)
    {
        // ��ȡң��������
        const int16_t *remote_ch = get_remote_ch_point();
        
        // ���õ��̿���ָ�ʹ��ң������ͨ��0��ͨ��3
        chassis_set_control(remote_ch[1], remote_ch[0]);
        
        // ִ�е��̿��Ƽ���
        chassis_control_calc();
        
        // ִ�е�����Ƽ���
        motor_control_calc();
        
        // ����Ƶ�ʣ�ͨ����ʱʵ��
        vTaskDelay(CAR_CONTROL_TIME_MS);
    }
}

/**
  * @brief          ����С�������߳�
  * @param[in]      none
  * @retval         none
  */
void car_task_create(void)
{
    // ����С����������
    xTaskCreate((TaskFunction_t)car_task,
                (const char *)"car_task",
                (uint16_t)CAR_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CAR_TASK_PRIO,
                (TaskHandle_t *)&car_task_handle);
}


