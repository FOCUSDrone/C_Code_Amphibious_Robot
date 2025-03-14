/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       bsp_can.h
  * @brief      CAN总线驱动，初始化CAN总线配置。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Claude          1. 重构优化
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"

/**
  * @brief          CAN总线滤波器初始化
  * @param[in]      none
  * @retval         none
  */
extern void can_filter_init(void);

#endif
