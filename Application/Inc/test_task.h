/**
  ********************************************************************************
  * @file       test_task.c/h
  * @brief      测试线程，写什么都可以,开启线程需要头文件定义宏TEST_TASK_WORK，
  *             会ban掉shoot和gimbal线程。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. 完成
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

#define TEST_TASK_WORK

#define TEST_TASK_TIME  1



#endif
