/**
  ******************************************************************************
  * @file       servo_receive.c/h
  * @brief      舵机回传接收，有usart1中断线程，有servo_receive线程
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

#ifndef SERVO_RECEIVE_H
#define SERVO_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"

#define SERVO_RECEIVE_TASK_TIME 10

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t load;
    int16_t voltage;
    int16_t temper;
    int16_t move;
    int16_t current;
} servo_measure_t;

extern servo_measure_t* get_servo_measure_point(uint8_t servo_id);

#endif
