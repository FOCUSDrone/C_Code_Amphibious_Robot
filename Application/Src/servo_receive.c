/**
  ******************************************************************************
  * @file       servo_receive.c/h
  * @brief      舵机回传接收，有usart1中断线程，有servo_receive线程
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
#include "servo_receive.h"
#include "detect_task.h"
#include "ft_servo_app.h"


extern UART_HandleTypeDef huart1;  /* 使用UART1与舵机通信 */
static int position=1;



void servo_receive_task(void const * argument)
{
	// 假设使用UART3连接舵机
	extern UART_HandleTypeDef huart1;

// 初始化舵机库
	ft_servo_app_init(&huart1);
    
    while(1) {
    ft_servo_app_wheel_mode(1);               // 设置为恒速模式
	ft_servo_app_write_speed(1, 500, 10);     // 以500速度正转
	position = ft_servo_app_read_pos(1);
	int speed = ft_servo_app_read_speed(1);
	int load = ft_servo_app_read_load(1);
	int voltage = ft_servo_app_read_voltage(1);
	int temperature = ft_servo_app_read_temperature(1);
	int moving = ft_servo_app_read_move(1);
	int current = ft_servo_app_read_current(1);
	


    }

}
