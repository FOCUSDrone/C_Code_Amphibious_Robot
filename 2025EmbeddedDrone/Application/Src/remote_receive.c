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

#include "remote_receive.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
static int16_t remote_ch[SBUS_RX_BUF_CH_NUM];

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_remote_ch(volatile const uint8_t *sbus_buf, int16_t *remote_ch_point);


//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            
            if(this_time_rx_len == AT9S_FRAME_LENGTH)
            {
                sbus_to_remote_ch(sbus_rx_buf[0], remote_ch);
                //记录数据接收时间
                detect_hook(DBUS_TOE);
//                sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == AT9S_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_remote_ch(sbus_rx_buf[1], remote_ch);
                //记录数据接收时间
                detect_hook(DBUS_TOE);
//                sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}


/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     remote_ch_point: 遥控器数据指
  * @retval         none
  */
static void sbus_to_remote_ch(volatile const uint8_t *sbus_buf, int16_t *remote_ch_point)
{
    if (sbus_buf == NULL || remote_ch_point == NULL)
    {
        return;
    }
    
    *(remote_ch_point+0) = ((int16_t)sbus_buf[1] >> 0 | ((int16_t)sbus_buf[2] << 8 )) & 0x07FF;
    *(remote_ch_point+1) = ((int16_t)sbus_buf[2] >> 3 | ((int16_t)sbus_buf[3] << 5 )) & 0x07FF;
    *(remote_ch_point+2) = ((int16_t)sbus_buf[3] >> 6 | ((int16_t)sbus_buf[4] << 2 )  | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;
    *(remote_ch_point+3) = ((int16_t)sbus_buf[5] >> 1 | ((int16_t)sbus_buf[6] << 7 )) & 0x07FF;
    *(remote_ch_point+4) = ((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF;
    *(remote_ch_point+5) = ((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 )  | (int16_t)sbus_buf[9] <<  9 ) & 0x07FF;
    *(remote_ch_point+6) = ((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF;
    *(remote_ch_point+7) = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF;
    
    *(remote_ch_point+8) = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
    *(remote_ch_point+9) = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
    *(remote_ch_point+10) = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 )  | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
    *(remote_ch_point+11) = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
    *(remote_ch_point+12) = ((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
    *(remote_ch_point+13) = ((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 )  | (int16_t)sbus_buf[20] <<  9 ) & 0x07FF;
    *(remote_ch_point+14) = ((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
    *(remote_ch_point+15) = ((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;
    
    *(remote_ch_point+0)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+1)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+2)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+3)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+4)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+5)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+6)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+7)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+8)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+9)  -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+10) -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+11) -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+12) -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+13) -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+14) -= REMOTE_CH_VALUE_OFFSET;
    *(remote_ch_point+15) -= REMOTE_CH_VALUE_OFFSET;
    
}


/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
static void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}


/**
  * @brief          获得遥控器通道数据指针
  * @param[in]      none
  * @retval         数据地址
  */
const int16_t *get_remote_ch_point(void)
{
    return remote_ch;
}


/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_receive_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    memset(remote_ch, 0, SBUS_RX_BUF_NUM);
}

