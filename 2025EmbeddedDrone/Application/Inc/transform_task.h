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
#define LEFT_SERVO_ID               1
#define RIGHT_SERVO_ID              2
#define TRANSFORM_REMOTE_CHANNEL    5
#define ALL_MOVE_ENABLE_CHANNEL     6

#define LEFT_SERVO_GROUND_POS       0
#define LEFT_SERVO_FLY_POS          0
#define RIGHT_SERVO_GROUND_POS      0
#define RIGHT_SERVO_FLY_POS         0

#define PUSH_ROG_ELONGATE_NEED_TIME    0
#define PUSH_ROG_SHORTEN_NEED_TIME     0

#define SERVO_SPEED     1000
#define SERVO_TIME      0

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
    const int16_t* remote_point;
    servo_measure_t* servo_left;
    servo_measure_t* servo_right;
    uint32_t push_rog_move_begin_time;
} transform_control_t;


#endif
  
