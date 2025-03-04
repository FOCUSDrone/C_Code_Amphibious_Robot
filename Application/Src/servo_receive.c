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

#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "servo_receive.h"
#include "detect_task.h"
#include "SCSCL.h"
#include "SCS.h"

extern UART_HandleTypeDef huart1;
uint8_t usart1_rx_buf[2][128];
static servo_measure_t servo_measure[4];


/**
  * @brief          servo_receive任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_receive_task(void const * argument)
{
    usart1_rx_dma_init(usart1_rx_buf[0], usart1_rx_buf[1], 128);
    setEnd(1);

    while(1)
    {
        uint8_t i = 0;
        for (i=0;i<4;i++){
            SCSCL_FeedBack(i+1);
            if (!getLastError()){
                servo_measure[i].pos     = SCSCL_ReadPos(-1);
                servo_measure[i].vel     = SCSCL_ReadSpeed(-1);
                servo_measure[i].load    = SCSCL_ReadLoad(-1);
                servo_measure[i].voltage = SCSCL_ReadVoltage(-1);
                servo_measure[i].temper  = SCSCL_ReadTemper(-1);
                servo_measure[i].move    = SCSCL_ReadMove(-1);
                vTaskDelay(SERVO_RECEIVE_TASK_TIME);
            }
        }
        
        vTaskDelay(SERVO_RECEIVE_TASK_TIME);
    }
}

void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = 128 - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, 128);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            detect_hook(SCSCL_SERVO_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = 128 - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, 128);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            detect_hook(SCSCL_SERVO_TOE);
        }
    }
}

servo_measure_t* get_servo_measure_point(uint8_t servo_id)
{
    if (servo_id > 4){
        return NULL;
    }
    
    return &servo_measure[servo_id-1];
}
