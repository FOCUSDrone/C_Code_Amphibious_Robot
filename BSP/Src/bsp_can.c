/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       bsp_can.c
  * @brief      CAN������������ʼ��CAN�������á�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Claude          1. �ع��Ż�
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#include "bsp_can.h"
#include "main.h"

/* �ⲿ�������� */
extern CAN_HandleTypeDef hcan1;

/**
  * @brief          CAN�����˲�����ʼ��
  * @param[in]      none
  * @retval         none
  */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    
    /* ����CAN�˲��� */
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    /* Ӧ��CAN�˲������� */
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    
    /* ����CANͨ�� */
    HAL_CAN_Start(&hcan1);
    
    /* ����CAN�����ж� */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

