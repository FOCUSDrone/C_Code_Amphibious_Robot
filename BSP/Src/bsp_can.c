/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       bsp_can.c
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

#include "bsp_can.h"
#include "main.h"

/* 外部变量声明 */
extern CAN_HandleTypeDef hcan1;

/**
  * @brief          CAN总线滤波器初始化
  * @param[in]      none
  * @retval         none
  */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    
    /* 配置CAN滤波器 */
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    /* 应用CAN滤波器配置 */
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    
    /* 启动CAN通信 */
    HAL_CAN_Start(&hcan1);
    
    /* 激活CAN接收中断 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

