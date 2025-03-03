#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "servo_receive.h"
#include "detect_task.h"

uint8_t usart1_rx_buf[2][128];

//´®¿ÚÖÐ¶Ï
extern UART_HandleTypeDef huart1;
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
