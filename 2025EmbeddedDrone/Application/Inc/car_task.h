/**
  ******************************************************************************
  * @file       car_task.c/h
  * @brief      路地行走线程，主要负责行走的航模电机驱动
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
  
#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "transform_task.h"
#include "remote_receive.h"
#include "CAN_receive.h"
#include "pid.h"

#define CAR_TASK_TIME  1

#define CH_TO_M2006_SPEED       0.0125f
#define LEFT_MOTOR_CH   2
#define RIGHT_MOTOR_CH  1

#define M2006_ANGLE_PID_KP        5.0f
#define M2006_ANGLE_PID_KI        0.0f
#define M2006_ANGLE_PID_KD        0.0f

#define M2006_SPEED_PID_KP        1000.0f
#define M2006_SPEED_PID_KI        0.0f
#define M2006_SPEED_PID_KD        0.0f

#define M2006_ANGLE_PID_MAX_OUT   10.0f
#define M2006_ANGLE_PID_MAX_IOUT  0.0f

#define M2006_SPEED_PID_MAX_OUT  3000.0f
#define M2006_SPEED_PID_MAX_IOUT 0.0f

//电机编码器数据转rad与rad/s
#define MOTOR_M2006_RPM_TO_SPEED      0.00290888208665721596153948461415f      //2 * PI / 60 / 36
#define M2006_HALF_ECD_RANGE          4096
#define M2006_ECD_RANGE               8191
#define MOTOR_M2006_ECD_TO_ANGLE      0.000021305288720633905968306772076277f  //2 * PI / 8191 / 36
#define FULL_COUNT                    18

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def angle_pid;
	pid_type_def speed_pid;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;
	fp32 current_set;
    int16_t given_current;
    int8_t ecd_count;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
} m2006_t;

typedef struct
{
    const int16_t* remote_point;
    const transform_control_t* transform_point;
    int16_t right_motor_ch;
    int16_t left_motor_ch;
    m2006_t left_motor;
    m2006_t right_motor;
    
} car_control_t;



#endif
