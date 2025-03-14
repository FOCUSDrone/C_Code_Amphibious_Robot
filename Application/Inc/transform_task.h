/**
  ******************************************************************************
  * @file       transform_task.c/h
  * @brief      变形线程，主要负责变形舵机和电推杆的驱动
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
  
#ifndef TRANSFORM_TASK_H
#define TRANSFORM_TASK_H

#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_receive.h"
#include "servo_receive.h"

#define TRANSFORM_TASK_TIME         1
#define FRONT_SERVO_ID              1
#define BEHIND_SERVO_ID             2
#define TRANSFORM_REMOTE_CHANNEL    7
#define ALL_MOVE_ENABLE_CHANNEL     6

#define FRONT_SERVO_GROUND_POS       3005
#define FRONT_SERVO_FLY_POS          3105
#define BEHIND_SERVO_GROUND_POS      3160
#define BEHIND_SERVO_FLY_POS         3260

#define PUSH_ROG_ELONGATE_NEED_TIME    8000
#define PUSH_ROG_SHORTEN_NEED_TIME     8000

#define SERVO_SPEED     1000
#define SERVO_ACC       50

#define SERVO_NORMAL_DELTA_POS  100
#define MAX_AGAINST_ERROR_POS   10

#define MAX_AGAINST_TIME    1000
#define NEED_REVERSE_TIME   2000

#define SERVO_MAX_LOAD  500

typedef enum
{
    TRANSFORM_INIT = 0,
    TRANSFORM_STOP,
    GROUND_STATE,
    GROUND_TRANSFORM_FLY_PUSH_ROD,
    GROUND_TRANSFORM_FLY_SERVO,
    FLY_STATE,
    FLY_TRANSFORM_GROUND_SERVO,
    FLY_TRANSFORM_GROUND_PUSH_ROD
} transform_state_e;


typedef struct
{
    transform_state_e transform_state;
    transform_state_e last_transform_state;
    transform_state_e before_disable_transform_state;
    const int16_t* remote_point;
    servo_measure_t* servo_front;
    servo_measure_t* servo_behind;
    uint32_t push_rog_move_begin_time;
} transform_control_t;


#endif
  
