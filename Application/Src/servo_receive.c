/**
  ******************************************************************************
  * @file       servo_receive.c/h
  * @brief      舵机回传接收
  * @note       
  * @history
  *  Version        Date        Author      Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "servo_receive.h"
#include "detect_task.h"
#include "ft_servo_app.h"

static servo_measure_t servo_measure[4];
extern UART_HandleTypeDef huart1;

/**
  * @brief          servo_receive任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_receive_task(void const * argument)
{
    ft_servo_app_init(&huart1);
    
    static servo_measure_t* point1 = &servo_measure[0];
    static servo_measure_t* point2 = &servo_measure[1];
    static servo_measure_t* point3 = &servo_measure[2];
    static servo_measure_t* point4 = &servo_measure[3];
    while(1)
    {
        uint8_t i = 0;
        for (i=0;i<4;i++){
            servo_measure[i].pos     = ft_servo_app_read_pos(i+1);
            servo_measure[i].vel     = ft_servo_app_read_speed(i+1);
            servo_measure[i].load    = ft_servo_app_read_load(i+1);
            servo_measure[i].voltage = ft_servo_app_read_voltage(i+1);
            servo_measure[i].temper  = ft_servo_app_read_temperature(i+1);
            servo_measure[i].move    = ft_servo_app_read_move(i+1);
            servo_measure[i].current = ft_servo_app_read_current(i+1);
        }
        
        vTaskDelay(SERVO_RECEIVE_TASK_TIME);
    }
}

servo_measure_t* get_servo_measure_point(uint8_t servo_id)
{
    if (servo_id > 4){
        return NULL;
    }
    
    return &servo_measure[servo_id-1];
}
