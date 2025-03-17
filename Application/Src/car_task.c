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

/**
  * @brief          С��������RTOS��ע��Ϊ�߳�
  * @param[in]      pvParameters: �̲߳���
  * @retval         none
  */
void car_task(void *pvParameters)
{
    motor_control_init();
    chassis_control_init();
    
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



