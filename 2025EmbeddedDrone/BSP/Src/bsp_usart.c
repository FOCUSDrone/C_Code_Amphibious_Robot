#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}


void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    //确保DMA被禁用
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    //清除DMA传输完成标志
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF1);

    //设置DMA外设地址
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    
    //设置内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //设置内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    
    //设置DMA数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //启用双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}


void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart1_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    usart1_tx_dma_enable(tx_buf, len);

}

void usart6_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    //确保DMA被禁用
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    //清除DMA传输完成标志
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    //设置DMA外设地址
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    
    //设置内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //设置内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    
    //设置DMA数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //启用双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void usart6_tx_dma_init(void)
{
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    //确保DMA被禁用
    while (hdma_usart6_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    //设置DMA外设地址
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}
