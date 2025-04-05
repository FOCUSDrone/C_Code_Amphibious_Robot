/**
  ******************************************************************************
  * @file       transform_task.c/h
  * @brief      变形线程，主要负责变形舵机和电推杆的驱动
  * @note       
  * @history
  *  Version        Date        Author      Modification
  *V2.0.0           4.5         Feiziben    真机实验
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
  
#include "transform_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "bsp_usart.h"
#include "servo_receive.h"
#include "remote_receive.h"
#include "bsp_push_rog.h"
#include "SCSCL.h"
#include "ft_servo_app.h"
#include "arm_math.h"
#include "user_lib.h"
#include <cmath>
#include <cstdio>

static transform_control_t transform_control;


/**
  * @brief          变形初始化
  * @param[in]      init 结构体控制块
  * @retval         none
  */
static void transform_init(transform_control_t* init);


/**
  * @brief          变形状态机设置
  * @param[in]      set_mode 结构体控制块
  * @retval         none
  */
static void transform_set_mode(transform_control_t* set_mode);


/**
  * @brief          变形控制
  * @param[in]      control 结构体控制块
  * @retval         none
  */
static void transform_set_control(transform_control_t* control);




/**
  * @brief          transform任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void transform_task(void const * argument)
{
    transform_init(&transform_control);
    
    
    
    
#ifdef TEST_TASK_WORK
    while (1)
    {
        vTaskDelay(TRANSFORM_TASK_TIME);
    }
#endif
    
    ft_servo_protect_enable(FRONT_SERVO_ID, 45);
    ft_servo_protect_enable(BEHIND_SERVO_ID, 45);
    ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
    ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
    left_push_rog_off();
    right_push_rog_off();
    
    
    while(1)
    {
        transform_set_mode(&transform_control);
        transform_set_control(&transform_control);
        vTaskDelay(TRANSFORM_TASK_TIME);
    }
}


/**
  * @brief          变形初始化
  * @param[in]      init 结构体控制块
  * @retval         none
  */
static void transform_init(transform_control_t* init)
{
    init->transform_state = TRANSFORM_INIT;
    init->last_transform_state = TRANSFORM_INIT;
    init->servo_front = get_servo_measure_point(FRONT_SERVO_ID);
    init->servo_behind = get_servo_measure_point(BEHIND_SERVO_ID);
    init->remote_point = get_remote_ch_point();
    init->push_rog_move_begin_time = 0;
    
}


/**
  * @brief          变形状态机设置，变形的自由度有两个，推杆和舵机
  * @param[in]      set_mode 结构体控制块
  * @retval         none
  */
static void transform_set_mode(transform_control_t* set_mode)
{
    transform_state_e state_cpy = set_mode->transform_state;
    uint32_t now_time = xTaskGetTickCount();
    /* disable */
    if (switch_is_down(set_mode->remote_point[ALL_MOVE_ENABLE_CHANNEL]))
    {
		if (set_mode->transform_state != TRANSFORM_STOP)
			set_mode->before_disable_transform_state = set_mode->transform_state;
        set_mode->transform_state = TRANSFORM_STOP;
    }
    /* disable to enable,recover state */
    else if (set_mode->transform_state == TRANSFORM_STOP)
    {
		set_mode->transform_state = set_mode->before_disable_transform_state;

    }
    /* init state */
    else if (set_mode->transform_state == TRANSFORM_INIT){
        set_mode->transform_state = FLY_TRANSFORM_GROUND_PUSH_ROD;
        set_mode->push_rog_move_begin_time = xTaskGetTickCount();
    }
    /* static state */
    else if (set_mode->transform_state == FLY_STATE ||
             set_mode->transform_state == GROUND_STATE)
    {
        if (switch_is_down(set_mode->remote_point[TRANSFORM_REMOTE_CHANNEL]) &&
            set_mode->transform_state == FLY_STATE)
        {
            set_mode->transform_state = FLY_TRANSFORM_GROUND_PUSH_ROD;
            set_mode->push_rog_move_begin_time = xTaskGetTickCount();
        }
        else if (switch_is_up(set_mode->remote_point[TRANSFORM_REMOTE_CHANNEL]) &&
                 set_mode->transform_state == GROUND_STATE)
        {
            set_mode->transform_state = GROUND_TRANSFORM_FLY_PUSH_ROD;
            set_mode->push_rog_move_begin_time = xTaskGetTickCount();
        }
    }
    /* dynamic state */
    else if (set_mode->transform_state == GROUND_TRANSFORM_FLY_PUSH_ROD ||
             set_mode->transform_state == GROUND_TRANSFORM_FLY_SERVO    ||
             set_mode->transform_state == FLY_TRANSFORM_GROUND_SERVO    ||
             set_mode->transform_state == FLY_TRANSFORM_GROUND_PUSH_ROD)
    {
        if (set_mode->transform_state == GROUND_TRANSFORM_FLY_PUSH_ROD &&
            now_time > (set_mode->push_rog_move_begin_time +
            PUSH_ROG_ELONGATE_NEED_TIME))
        {
            set_mode->transform_state = GROUND_TRANSFORM_FLY_SERVO;
        }
        else if (set_mode->transform_state == GROUND_TRANSFORM_FLY_SERVO &&
                 abs(set_mode->servo_front->pos - FRONT_SERVO_FLY_POS) < 70 &&
                 abs(set_mode->servo_behind->pos - BEHIND_SERVO_FLY_POS)<70)
        {
            set_mode->transform_state = FLY_STATE;
        }
        else if (set_mode->transform_state == FLY_TRANSFORM_GROUND_SERVO &&
                 abs(set_mode->servo_front->pos - FRONT_SERVO_GROUND_POS) < 70  &&
                 abs(set_mode->servo_behind->pos - BEHIND_SERVO_GROUND_POS) < 70)
        {
            set_mode->transform_state = GROUND_STATE;
            
        }
        else if (set_mode->transform_state == FLY_TRANSFORM_GROUND_PUSH_ROD &&
                 now_time > (set_mode->push_rog_move_begin_time +
                 PUSH_ROG_SHORTEN_NEED_TIME))
        {
            set_mode->transform_state = FLY_TRANSFORM_GROUND_SERVO;
        }
    }
    
    set_mode->last_transform_state = state_cpy;
}

/**
  * @brief          变形控制
  * @param[in]      control 结构体控制块
  * @retval         none
  */
static void transform_set_control(transform_control_t* control)
{
    switch(control->transform_state){
    case GROUND_STATE:
		shorten_left_push_rog();
		shorten_right_push_rog();
		ft_servo_app_torque_enable(FRONT_SERVO_ID, 1);
        ft_servo_app_torque_enable(BEHIND_SERVO_ID, 1);
		break;
    case FLY_STATE:
		elongate_left_push_rog();
		elongate_right_push_rog();
		ft_servo_app_torque_enable(FRONT_SERVO_ID, 1);
        ft_servo_app_torque_enable(BEHIND_SERVO_ID, 1);
		break;
    case TRANSFORM_STOP:
		ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
        ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
        left_push_rog_off();
        right_push_rog_off();
		break;
	case TRANSFORM_INIT:
        ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
        ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
        left_push_rog_off();
        right_push_rog_off();
        break;
    case FLY_TRANSFORM_GROUND_SERVO:
        if (control->last_transform_state != FLY_TRANSFORM_GROUND_SERVO){
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 1);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 1);
			shorten_left_push_rog();
            shorten_right_push_rog();
            ft_servo_app_write_pos(FRONT_SERVO_ID, FRONT_SERVO_GROUND_POS,
                                   SERVO_SPEED, SERVO_ACC);
            ft_servo_app_write_pos(BEHIND_SERVO_ID, BEHIND_SERVO_GROUND_POS,
                                   SERVO_SPEED, SERVO_ACC);
        }
        break;
    case GROUND_TRANSFORM_FLY_SERVO:
        if (control->last_transform_state != GROUND_TRANSFORM_FLY_SERVO){
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 1);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 1);
			elongate_left_push_rog();
            elongate_right_push_rog();
            ft_servo_app_write_pos(FRONT_SERVO_ID, FRONT_SERVO_FLY_POS,
                                   SERVO_SPEED, SERVO_ACC);
            ft_servo_app_write_pos(BEHIND_SERVO_ID, BEHIND_SERVO_FLY_POS,
                                   SERVO_SPEED, SERVO_ACC);
        }
        break;
    case FLY_TRANSFORM_GROUND_PUSH_ROD:
        if (control->last_transform_state != FLY_TRANSFORM_GROUND_PUSH_ROD){
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
            shorten_left_push_rog();
            shorten_right_push_rog();
        }
        break;
    case GROUND_TRANSFORM_FLY_PUSH_ROD:
        if (control->last_transform_state != GROUND_TRANSFORM_FLY_PUSH_ROD){
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
            elongate_left_push_rog();
            elongate_right_push_rog();
        }
        break;
    }
}