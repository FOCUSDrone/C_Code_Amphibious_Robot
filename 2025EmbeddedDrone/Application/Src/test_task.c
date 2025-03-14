/**
  ******************************************************************************
  * @file       test_task.c/h
  * @brief      �����̣߳�дʲô������,�����߳���Ҫͷ�ļ������TEST_TASK_WORK��
  *             ��ban��transform��car�̡߳�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. ���
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
#include "pid.h"
#include "car_task.h"

//��������
const key_data_t *key_data_point;
uint8_t test_short_press_cnt;
uint8_t test_long_press_cnt;

static bool_t key_flip_state;

extern uint8_t **usart1_rx_buf;


static m2006_test_t m2006_test;
static void m2006_test_init(m2006_test_t *m2006_test);
static void m2006_feedback_update(void);
static void m2006_test_loop(void);


/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    m2006_test_init(&m2006_test);
    
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
        
//        //�̰�
//        if(key_data_point->short_press_cnt > test_short_press_cnt){
//            test_short_press_cnt = key_data_point->short_press_cnt;
//            
//            left_push_rog_off();
//            right_push_rog_off();
//            ft_servo_app_torque_enable(2, 1);
//        }
//        //����
//        if(key_data_point->long_press_cnt > test_long_press_cnt){
//            test_long_press_cnt = key_data_point->long_press_cnt;
//            
//            if (key_flip_state == 0){
//                elongate_left_push_rog();
//                elongate_right_push_rog();
//                key_flip_state = 1;
//            }
//            else if (key_flip_state == 1){
//                shorten_left_push_rog();
//                shorten_right_push_rog();
//                key_flip_state = 0;
//            }
//        }
        
        m2006_feedback_update(); //��������
        
        //�̰�
        if(key_data_point->short_press_cnt > test_short_press_cnt){
            test_short_press_cnt = key_data_point->short_press_cnt;
            
            //���̵��
            //����Ƕ��ж�
            if (rad_format(m2006_test.angle_set - m2006_test.angle) < 0.05f)
            {
                m2006_test.move_flag = 0;
            }
            //ÿ�β��� 1/4PI�ĽǶ�
            if (m2006_test.move_flag == 0)
            {
                m2006_test.angle_set = rad_format(m2006_test.angle + PI_FOUR);
                m2006_test.move_flag = 1;
            }
        }
        //����
        if(key_data_point->long_press_cnt > test_long_press_cnt){
            test_long_press_cnt = key_data_point->long_press_cnt;
            
            
        }
        
        m2006_test_loop();
        
        vTaskDelay(TEST_TASK_TIME);
    }
}


static void m2006_test_init(m2006_test_t *m2006_test){
    static const fp32 Left_angle_pid[3] = {M2006_ANGLE_PID_KP, M2006_ANGLE_PID_KI, M2006_ANGLE_PID_KD};
	static const fp32 Left_speed_pid[3] = {M2006_SPEED_PID_KP, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD};
    //���ָ��
    m2006_test->motor_measure = get_left_motor_measure_point();
    //��ʼ��PID
    PID_init(&m2006_test->angle_pid, PID_POSITION, Left_angle_pid, M2006_ANGLE_PID_MAX_OUT, M2006_ANGLE_PID_MAX_IOUT);
	PID_init(&m2006_test->speed_pid, PID_POSITION, Left_speed_pid, M2006_SPEED_PID_MAX_OUT, M2006_SPEED_PID_MAX_IOUT);
    //��������
    m2006_feedback_update();
    m2006_test->given_current = 0;
    m2006_test->move_flag = 0;
    m2006_test->angle_set = m2006_test->angle;
    m2006_test->speed = 0.0f;
    m2006_test->speed_set = 0.0f;
    m2006_test->block_time = 0;
    m2006_test->reverse_time = 0;
    m2006_test->ecd_count = 0;
}


static void m2006_test_loop(void){
    //����PID����
    fp32 error_fp32 = loop_fp32_constrain(m2006_test.angle_set - m2006_test.angle, -PI, PI);
    m2006_test.speed_set = PID_calc_by_error(&m2006_test.angle_pid, error_fp32);
    m2006_test.current_set = PID_calc(&m2006_test.speed_pid, m2006_test.speed, m2006_test.speed_set);
    m2006_test.given_current = (int16_t)(m2006_test.current_set);
    
    //���Ϳ���ָ��
    taskENTER_CRITICAL();
    CAN_cmd(m2006_test.given_current, m2006_test.given_current);
    taskEXIT_CRITICAL();
}


static void m2006_feedback_update(void){
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (m2006_test.motor_measure->int16_vel * MOTOR_M2006_RPM_TO_SPEED) * fliter_num[2];
    m2006_test.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (m2006_test.motor_measure->u16_pos - m2006_test.motor_measure->u16_last_pos > M2006_HALF_ECD_RANGE)
    {
        m2006_test.ecd_count--;
    }
    else if (m2006_test.motor_measure->u16_pos - m2006_test.motor_measure->u16_last_pos < -M2006_HALF_ECD_RANGE)
    {
        m2006_test.ecd_count++;
    }

    if (m2006_test.ecd_count == FULL_COUNT)
    {
        m2006_test.ecd_count = -FULL_COUNT;
    }
    else if (m2006_test.ecd_count == -(FULL_COUNT+1))
    {
        m2006_test.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    m2006_test.angle = (m2006_test.ecd_count * M2006_ECD_RANGE + m2006_test.motor_measure->u16_pos) * MOTOR_M2006_ECD_TO_ANGLE;
}
