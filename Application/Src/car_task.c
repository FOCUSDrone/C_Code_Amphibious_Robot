/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       car_task.c
  * @brief      差速小车控制任务，通过遥控器控制两个RM2006电机实现差速转向。
  * @note       作为独立RTOS线程实现
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Feiziben          1. 完成
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
  * @brief          小车任务，在RTOS中注册为线程
  * @param[in]      pvParameters: 线程参数
  * @retval         none
  */
void car_task(void *pvParameters)
{
    motor_control_init();
    chassis_control_init();
    
    while (1)
    {
        // 获取遥控器数据
        const int16_t *remote_ch = get_remote_ch_point();
        
        // 设置底盘控制指令，使用遥控器的通道0和通道3
        chassis_set_control(remote_ch[1], remote_ch[0]);
        
        // 执行底盘控制计算
        chassis_control_calc();
        
        // 执行电机控制计算
        motor_control_calc();
        
        // 控制频率，通过延时实现
        vTaskDelay(CAR_CONTROL_TIME_MS);
    }
}



