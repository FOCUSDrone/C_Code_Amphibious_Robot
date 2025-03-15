/**
  ******************************************************************************
  * @file       transform_task.c/h
  * @brief      变形线程，主要负责变形舵机和电推杆的驱动
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
  * @brief          判断是否左右手互博
  * @param[in]      is_against 结构体控制块
  * @retval         none
  */
static bool_t is_left_hand_against_right_hand(transform_control_t* is_against);


/**
  * @brief          transform任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void transform_task(void const * argument)
{
    transform_init(&transform_control);
    
    
    while (toe_is_error(SCSCL_SERVO_TOE))
    {
        vTaskDelay(TRANSFORM_TASK_TIME);
    }
    
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
    if (switch_is_down(set_mode->remote_point[ALL_MOVE_ENABLE_CHANNEL]) ||
        is_left_hand_against_right_hand(set_mode))
    {
        set_mode->before_disable_transform_state = set_mode->transform_state;
        set_mode->transform_state = TRANSFORM_STOP;
    }
    /* disable to enable,recover state */
    else if (set_mode->transform_state == TRANSFORM_STOP)
    {
        if (switch_is_up(set_mode->remote_point[TRANSFORM_REMOTE_CHANNEL])){
            switch(set_mode->before_disable_transform_state){
            case FLY_STATE:
                set_mode->transform_state = FLY_STATE;
                break;
            case GROUND_TRANSFORM_FLY_PUSH_ROD:
            case FLY_TRANSFORM_GROUND_PUSH_ROD:
            case GROUND_STATE:
                set_mode->transform_state = GROUND_TRANSFORM_FLY_PUSH_ROD;
                break;
            case GROUND_TRANSFORM_FLY_SERVO:
            case FLY_TRANSFORM_GROUND_SERVO:
                set_mode->transform_state = GROUND_TRANSFORM_FLY_SERVO;
                break;
            default:
                break;
            }
        }
        
        else if (switch_is_down(set_mode->remote_point[TRANSFORM_REMOTE_CHANNEL])){
            switch(set_mode->before_disable_transform_state){
            case GROUND_STATE:
                set_mode->transform_state = GROUND_STATE;
                break;
            case FLY_TRANSFORM_GROUND_SERVO:
            case GROUND_TRANSFORM_FLY_SERVO:
            case FLY_STATE:
                set_mode->transform_state = FLY_TRANSFORM_GROUND_SERVO;
                break;
            case FLY_TRANSFORM_GROUND_PUSH_ROD:
            case GROUND_TRANSFORM_FLY_PUSH_ROD:
                set_mode->transform_state = FLY_TRANSFORM_GROUND_PUSH_ROD;
                break;
            default:
                break;
            }
        }
    }
    /* init state */
    else if (set_mode->transform_state == TRANSFORM_INIT){
        set_mode->transform_state = FLY_TRANSFORM_GROUND_SERVO;
    }
    /* static state */
    else if (set_mode->transform_state == FLY_STATE ||
             set_mode->transform_state == GROUND_STATE)
    {
        if (switch_is_down(set_mode->remote_point[TRANSFORM_REMOTE_CHANNEL]) &&
            set_mode->transform_state == FLY_STATE)
        {
            set_mode->transform_state = FLY_TRANSFORM_GROUND_SERVO;
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
                 set_mode->servo_front->move == 0 &&
                 set_mode->servo_front->pos == FRONT_SERVO_FLY_POS &&
                 set_mode->servo_behind->move == 0 &&
                 set_mode->servo_behind->pos == BEHIND_SERVO_FLY_POS)
        {
            set_mode->transform_state = FLY_STATE;
        }
        else if (set_mode->transform_state == FLY_TRANSFORM_GROUND_SERVO &&
                 set_mode->servo_front->move == 0 &&
                 set_mode->servo_front->pos == FRONT_SERVO_GROUND_POS &&
                 set_mode->servo_behind->move == 0 &&
                 set_mode->servo_behind->pos == BEHIND_SERVO_GROUND_POS)
        {
            set_mode->transform_state = FLY_TRANSFORM_GROUND_PUSH_ROD;
            set_mode->push_rog_move_begin_time = xTaskGetTickCount();
        }
        else if (set_mode->transform_state == FLY_TRANSFORM_GROUND_PUSH_ROD &&
                 now_time > (set_mode->push_rog_move_begin_time +
                 PUSH_ROG_SHORTEN_NEED_TIME))
        {
            set_mode->transform_state = GROUND_STATE;
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
    case FLY_STATE:
    case TRANSFORM_STOP:
    case TRANSFORM_INIT:
        left_push_rog_off();
        right_push_rog_off();
        ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
        ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
        break;
    case FLY_TRANSFORM_GROUND_SERVO:
        if (control->last_transform_state != FLY_TRANSFORM_GROUND_SERVO){
            left_push_rog_off();
            right_push_rog_off();
            ft_servo_app_write_pos(FRONT_SERVO_ID, FRONT_SERVO_GROUND_POS,
                                   SERVO_SPEED, SERVO_ACC);
            ft_servo_app_write_pos(BEHIND_SERVO_ID, FRONT_SERVO_GROUND_POS,
                                   SERVO_SPEED, SERVO_ACC);
        }
        break;
    case GROUND_TRANSFORM_FLY_SERVO:
        if (control->last_transform_state != GROUND_TRANSFORM_FLY_SERVO){
            left_push_rog_off();
            right_push_rog_off();
            ft_servo_app_write_pos(FRONT_SERVO_ID, FRONT_SERVO_FLY_POS,
                                   SERVO_SPEED, SERVO_ACC);
            ft_servo_app_write_pos(BEHIND_SERVO_ID, BEHIND_SERVO_FLY_POS,
                                   SERVO_SPEED, SERVO_ACC);
        }
        break;
    case FLY_TRANSFORM_GROUND_PUSH_ROD:
        if (control->last_transform_state != FLY_TRANSFORM_GROUND_PUSH_ROD){
            shorten_left_push_rog(PUSH_ROG_ON_PWM);
            shorten_right_push_rog(PUSH_ROG_ON_PWM);
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
        }
        break;
    case GROUND_TRANSFORM_FLY_PUSH_ROD:
        if (control->last_transform_state != GROUND_TRANSFORM_FLY_PUSH_ROD){
            elongate_left_push_rog(PUSH_ROG_ON_PWM);
            elongate_right_push_rog(PUSH_ROG_ON_PWM);
            ft_servo_app_torque_enable(FRONT_SERVO_ID, 0);
            ft_servo_app_torque_enable(BEHIND_SERVO_ID, 0);
        }
        break;
    }
}


/**
  * @brief          判断是否左右手互博
  * @param[in]      is_against 结构体控制块
  * @retval         none
  */
static bool_t is_left_hand_against_right_hand(transform_control_t* is_against){
    static uint16_t against_time = 0;
    static uint16_t reverse_time = 0;
    bool_t ret = 0;
    fp32 error_pos = 0;
    
    if (against_time > MAX_AGAINST_TIME ||
        against_time == MAX_AGAINST_TIME)
    {
        ret = 1;
    }
    
    if ((int16_abs(is_against->servo_front->load) > SERVO_MAX_LOAD ||
        int16_abs(is_against->servo_behind->load) > SERVO_MAX_LOAD)&&
        against_time < MAX_AGAINST_TIME)
    {
        against_time++;
        reverse_time = 0;
    }
    else if (against_time == MAX_AGAINST_TIME && reverse_time < NEED_REVERSE_TIME){
        reverse_time++;
    }
    else {
        against_time = 0;
    }
    
    return ret;
}

//if( shoot_control.block_time < BLOCK_TIME)
//{
//    shoot_control.speed_set = shoot_control.trigger_speed_set;
//}
//else
//{
//    shoot_control.speed_set = -shoot_control.trigger_speed_set;
//}

//if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//{
//    shoot_control.block_time++;
//    shoot_control.reverse_time = 0;
//}
//else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//{
//    shoot_control.reverse_time++;
//}
//else
//{
//    shoot_control.block_time = 0;
//}
