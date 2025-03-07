/**
  ******************************************************************************
  * @file       servo_receive.c/h
  * @brief      ����ش����գ���usart1�ж��̣߳���servo_receive�߳�
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


extern UART_HandleTypeDef huart1;  /* ʹ��UART1����ͨ�� */
static int position=1;



void servo_receive_task(void const * argument)
{
	// ����ʹ��UART3���Ӷ��
	extern UART_HandleTypeDef huart1;

// ��ʼ�������
	ft_servo_app_init(&huart1);
    
    while(1) {
    ft_servo_app_wheel_mode(1);               // ����Ϊ����ģʽ
	ft_servo_app_write_speed(1, 500, 10);     // ��500�ٶ���ת
	position = ft_servo_app_read_pos(1);
	int speed = ft_servo_app_read_speed(1);
	int load = ft_servo_app_read_load(1);
	int voltage = ft_servo_app_read_voltage(1);
	int temperature = ft_servo_app_read_temperature(1);
	int moving = ft_servo_app_read_move(1);
	int current = ft_servo_app_read_current(1);
	


    }

}
