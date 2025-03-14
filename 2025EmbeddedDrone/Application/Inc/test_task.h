/**
  ********************************************************************************
  * @file       test_task.c/h
  * @brief      �����̣߳�дʲô������,�����߳���Ҫͷ�ļ������TEST_TASK_WORK��
  *             ��ban��shoot��gimbal�̡߳�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ********************************************************************************
  */

#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "bsp_buzzer.h"
#include "remote_receive.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "key_task.h"
#include "user_lib.h"
#include "pid.h"

//#define TEST_TASK_WORK

#define TEST_TASK_TIME  1

//PI����
#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

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
} m2006_test_t;

#endif
