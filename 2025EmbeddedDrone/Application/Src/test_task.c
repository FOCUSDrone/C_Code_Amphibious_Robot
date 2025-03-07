/**
  ******************************************************************************
  * @file       test_task.c/h
  * @brief      测试线程，写什么都可以,开启线程需要头文件定义宏TEST_TASK_WORK，
  *             会ban掉transform和car线程。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "bsp_buzzer.h"
#include "remote_receive.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "key_task.h"
#include "user_lib.h"
#include "SCS.h"
#include "SCSCL.h"
#include "ft_servo_app.h"
#include "SMS_STS.h"
#include "bsp_push_rog.h"

//按键输入
const key_data_t *key_data_point;
uint8_t test_short_press_cnt;
uint8_t test_long_press_cnt;

static bool_t key_flip_state;

extern uint8_t **usart1_rx_buf;

/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    key_data_point = get_key_data_point();
    test_short_press_cnt = 0;
    test_long_press_cnt = 0;
    
#ifndef TEST_TASK_WORK
    
    while(1)
    {
        vTaskDelay(TEST_TASK_TIME);
    }
    
#endif

    while(1)
    {
        
        //短按
        if(key_data_point->short_press_cnt > test_short_press_cnt){
            test_short_press_cnt = key_data_point->short_press_cnt;
            
            left_push_rog_off();
            right_push_rog_off();
            
//            test1 = ft_servo_app_torque_enable(1, 0);
            ft_servo_app_torque_enable(2, 1);
//            ft_servo_app_torque_enable(3, 0);
//            ft_servo_app_torque_enable(4, 0);
//            ft_servo_app_write_pos(1, 0, 0, 0);
//            ft_servo_app_write_speed(1, 0, 0);
//            ft_servo_app_write_pos(1, 1000, 1500, 200);
//            SCSCL_WritePos(1, 1000, 0, 1500);
        }
        //长按
        if(key_data_point->long_press_cnt > test_long_press_cnt){
            test_long_press_cnt = key_data_point->long_press_cnt;
            
            if (key_flip_state == 0){
                elongate_left_push_rog(PUSH_ROG_ON_PWM);
                elongate_right_push_rog(PUSH_ROG_ON_PWM);
                key_flip_state = 1;
            }
            else if (key_flip_state == 1){
                shorten_left_push_rog(PUSH_ROG_ON_PWM);
                shorten_right_push_rog(PUSH_ROG_ON_PWM);
                key_flip_state = 0;
            }
//            ft_servo_app_write_speed(1, 1000, 100);
//            ft_servo_app_write_pos(1, 2000, 600, 10);
 
        }
        
        
        
        vTaskDelay(TEST_TASK_TIME);
    }
}


