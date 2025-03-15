/**
  ******************************************************************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef REMOTE_RECEIVE_H
#define REMOTE_RECEIVE_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM         36u
#define SBUS_RX_BUF_CH_NUM      16u
#define AT9S_FRAME_LENGTH       25u

#define REMOTE_CH_VALUE_MIN     ((uint16_t)200)
#define REMOTE_CH_VALUE_OFFSET  ((uint16_t)1000)
#define REMOTE_CH_VALUE_MAX     ((uint16_t)1800)

#define REMOTE_DEADBAND         50    

#define REMOTE_SW_UP_RANGE                ((int16_t)700)
#define REMOTE_SW_MID_RANGE               ((int16_t)100)
#define REMOTE_SW_DOWN_RANGE              ((int16_t)-700)
#define switch_is_down(ch)       ((ch) < REMOTE_SW_DOWN_RANGE)
#define switch_is_mid(ch)        ((ch) > -REMOTE_SW_MID_RANGE && (ch) < REMOTE_SW_MID_RANGE)
#define switch_is_up(ch)         ((ch) > REMOTE_SW_UP_RANGE)

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define remote_deadband_limit(input, output, dealine)    \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

extern void remote_receive_init(void);
extern const int16_t *get_remote_ch_point(void);
extern void sbus_to_usart1(uint8_t *sbus);
#endif
