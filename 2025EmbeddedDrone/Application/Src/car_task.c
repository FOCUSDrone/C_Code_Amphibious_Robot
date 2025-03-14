/**
  ******************************************************************************
  * @file       car_task.c/h
  * @brief      ·�������̣߳���Ҫ�������ߵĺ�ģ�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
  
#include "car_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "user_lib.h"
#include "remote_receive.h"
#include "bsp_car.h"
#include "transform_task.h"
#include "detect_task.h"
#include "user_lib.h"
#include "pid.h"
#include "CAN_receive.h"


car_control_t car_control;
static void car_init(void);
static void car_feedback_update(void);

/**
  * @brief          car����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void car_task(void const * argument)
{
    car_init();
    
#ifdef TEST_TASK_WORK
    while (1)
    {
        vTaskDelay(CAR_TASK_TIME);
    }
#endif
    
    while(1)
    {
        car_feedback_update(); //��������
        
        car_control.left_motor.speed_set = car_control.left_motor_ch * CH_TO_M2006_SPEED;
        car_control.right_motor.speed_set = car_control.right_motor_ch * CH_TO_M2006_SPEED;
        
        //�ٶȻ�PID����
        car_control.left_motor.current_set = PID_calc(&car_control.left_motor.speed_pid, car_control.left_motor.speed, car_control.left_motor.speed_set);
        car_control.left_motor.given_current = (int16_t)(car_control.left_motor.current_set);
        car_control.right_motor.current_set = PID_calc(&car_control.right_motor.speed_pid, car_control.right_motor.speed, car_control.right_motor.speed_set);
        car_control.right_motor.given_current = (int16_t)(car_control.right_motor.current_set);
        
        if (!toe_is_error(DBUS_TOE))
        {
            taskENTER_CRITICAL();
            CAN_cmd(car_control.left_motor.given_current, car_control.right_motor.given_current);
            taskEXIT_CRITICAL();
        }
        else {
            taskENTER_CRITICAL();
            CAN_cmd(0, 0);
            taskEXIT_CRITICAL();
        }
        
//        car_control.left_motor_pwm_set = int16_abs(car_control.left_motor_ch) * CH_TO_PWM;
//        car_control.right_motor_pwm_set = int16_abs(car_control.right_motor_ch) * CH_TO_PWM;
//        car_left_motor_on(car_control.left_motor_pwm_set);
//        car_right_motor_on(car_control.right_motor_pwm_set);
        
//        if (car_control.transform_point->transform_state == GROUND_STATE &&
//            !toe_is_error(DBUS_TOE))
//        {
//            car_left_motor_on(car_control.left_motor_pwm_set);
//            car_right_motor_on(car_control.right_motor_pwm_set);
//        }
//        else {
//            car_left_motor_off();
//            car_right_motor_off();
//        }
        
        vTaskDelay(CAR_TASK_TIME);
    }
}

static void car_init(void){
    static const fp32 Angle_pid[3] = {M2006_ANGLE_PID_KP, M2006_ANGLE_PID_KI, M2006_ANGLE_PID_KD};
	static const fp32 Speed_pid[3] = {M2006_SPEED_PID_KP, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD};
    
    car_control.remote_point = get_remote_ch_point();
    car_control.transform_point = get_transform_point();
    car_control.left_motor_ch = 0;
    car_control.right_motor_ch = 0;
    
    //���ָ��
    car_control.left_motor.motor_measure = get_left_motor_measure_point();
    car_control.right_motor.motor_measure = get_right_motor_measure_point();
    //��ʼ��PID
    PID_init(&car_control.left_motor.angle_pid, PID_POSITION, Angle_pid, M2006_ANGLE_PID_MAX_OUT, M2006_ANGLE_PID_MAX_IOUT);
	PID_init(&car_control.left_motor.speed_pid, PID_POSITION, Speed_pid, M2006_SPEED_PID_MAX_OUT, M2006_SPEED_PID_MAX_IOUT);
    PID_init(&car_control.right_motor.angle_pid, PID_POSITION, Angle_pid, M2006_ANGLE_PID_MAX_OUT, M2006_ANGLE_PID_MAX_IOUT);
	PID_init(&car_control.right_motor.speed_pid, PID_POSITION, Speed_pid, M2006_SPEED_PID_MAX_OUT, M2006_SPEED_PID_MAX_IOUT);
    //��������
    car_feedback_update();
    car_control.left_motor.given_current = 0;
    car_control.left_motor.move_flag = 0;
    car_control.left_motor.angle_set = car_control.left_motor.angle;
    car_control.left_motor.speed = 0.0f;
    car_control.left_motor.speed_set = 0.0f;
    car_control.left_motor.block_time = 0;
    car_control.left_motor.reverse_time = 0;
    car_control.left_motor.ecd_count = 0;
    
    car_control.right_motor.given_current = 0;
    car_control.right_motor.move_flag = 0;
    car_control.right_motor.angle_set = car_control.left_motor.angle;
    car_control.right_motor.speed = 0.0f;
    car_control.right_motor.speed_set = 0.0f;
    car_control.right_motor.block_time = 0;
    car_control.right_motor.reverse_time = 0;
    car_control.right_motor.ecd_count = 0;
}

static void car_feedback_update(void){
    static fp32 left_speed_fliter_1 = 0.0f;
    static fp32 left_speed_fliter_2 = 0.0f;
    static fp32 left_speed_fliter_3 = 0.0f;
    static fp32 right_speed_fliter_1 = 0.0f;
    static fp32 right_speed_fliter_2 = 0.0f;
    static fp32 right_speed_fliter_3 = 0.0f;
    
    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    left_speed_fliter_1 = left_speed_fliter_2;
    left_speed_fliter_2 = left_speed_fliter_3;
    left_speed_fliter_3 = left_speed_fliter_2 * fliter_num[0] + left_speed_fliter_1 * fliter_num[1] 
        + (car_control.left_motor.motor_measure->int16_vel * MOTOR_M2006_RPM_TO_SPEED) * fliter_num[2];
    
    right_speed_fliter_1 = right_speed_fliter_2;
    right_speed_fliter_2 = right_speed_fliter_3;
    right_speed_fliter_3 = right_speed_fliter_2 * fliter_num[0] + right_speed_fliter_1 * fliter_num[1] 
        + (car_control.right_motor.motor_measure->int16_vel * MOTOR_M2006_RPM_TO_SPEED) * fliter_num[2];
    
    car_control.left_motor.speed = left_speed_fliter_3;
    car_control.right_motor.speed = right_speed_fliter_3;
    
    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (car_control.left_motor.motor_measure->u16_pos - car_control.left_motor.motor_measure->u16_last_pos > M2006_HALF_ECD_RANGE)
    {
        car_control.left_motor.ecd_count--;
    }
    else if (car_control.left_motor.motor_measure->u16_pos - car_control.left_motor.motor_measure->u16_last_pos < -M2006_HALF_ECD_RANGE)
    {
        car_control.left_motor.ecd_count++;
    }

    if (car_control.left_motor.ecd_count == FULL_COUNT)
    {
        car_control.left_motor.ecd_count = -FULL_COUNT;
    }
    else if (car_control.left_motor.ecd_count == -(FULL_COUNT+1))
    {
        car_control.left_motor.ecd_count = FULL_COUNT - 1;
    }
    
    if (car_control.right_motor.motor_measure->u16_pos - car_control.right_motor.motor_measure->u16_last_pos > M2006_HALF_ECD_RANGE)
    {
        car_control.right_motor.ecd_count--;
    }
    else if (car_control.right_motor.motor_measure->u16_pos - car_control.right_motor.motor_measure->u16_last_pos < -M2006_HALF_ECD_RANGE)
    {
        car_control.right_motor.ecd_count++;
    }

    if (car_control.right_motor.ecd_count == FULL_COUNT)
    {
        car_control.right_motor.ecd_count = -FULL_COUNT;
    }
    else if (car_control.right_motor.ecd_count == -(FULL_COUNT+1))
    {
        car_control.right_motor.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    car_control.left_motor.angle = (car_control.left_motor.ecd_count * M2006_ECD_RANGE + car_control.left_motor.motor_measure->u16_pos) * MOTOR_M2006_ECD_TO_ANGLE;
    car_control.right_motor.angle = (car_control.left_motor.ecd_count * M2006_ECD_RANGE + car_control.left_motor.motor_measure->u16_pos) * MOTOR_M2006_ECD_TO_ANGLE;
    
    //ң����
    remote_deadband_limit(car_control.remote_point[LEFT_MOTOR_CH], car_control.left_motor_ch, REMOTE_DEADBAND);
    remote_deadband_limit(car_control.remote_point[RIGHT_MOTOR_CH], car_control.right_motor_ch, REMOTE_DEADBAND);
}

