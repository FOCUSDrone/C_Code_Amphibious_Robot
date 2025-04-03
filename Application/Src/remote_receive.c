/**
  ******************************************************************************
  * @file       remote_control.c
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. 完成
  *  V1.0.1     2025.3.27       Feiziben        2. 优化帧验证和错误处理
  *	 V1.0.2     2025.4.03       Sir0505         3.解决遥控关机后风扇失控的问题
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
  * @brief          验证SBUS帧的完整性
  * @param[in]      sbus_buf: 包含SBUS数据的缓冲区
  * @param[in]      len: 接收数据的长度
  * @retval         如果是有效帧则为1，否则为0
  */
static uint8_t validate_sbus_frame(const uint8_t *sbus_buf, uint16_t len)
{
    if (sbus_buf == NULL)
    {
        return 0;
    }
    
    // 基本长度检查
    if (len != AT9S_FRAME_LENGTH)
    {
        return 0;
    }
    
    // 可以添加更多的协议验证，如起始字节检查
    // 例如：if (sbus_buf[0] != 0x0F) return 0;
    
    return 1;
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     remote_ch_point: 遥控器数据指针
  * @retval         none
  */
static void sbus_to_remote_ch(volatile const uint8_t *sbus_buf, int16_t *remote_ch_point)
{
    if (sbus_buf == NULL || remote_ch_point == NULL)
    {
        return;
    }
    
    // 临时缓存，避免部分更新
    int16_t temp_ch[SBUS_RX_BUF_CH_NUM];
    
    // 保持原始的位操作方式，确保功能一致性
    temp_ch[0] = ((int16_t)sbus_buf[1] >> 0 | ((int16_t)sbus_buf[2] << 8 )) & 0x07FF;
    temp_ch[1] = ((int16_t)sbus_buf[2] >> 3 | ((int16_t)sbus_buf[3] << 5 )) & 0x07FF;
    temp_ch[2] = ((int16_t)sbus_buf[3] >> 6 | ((int16_t)sbus_buf[4] << 2 )  | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;
    temp_ch[3] = ((int16_t)sbus_buf[5] >> 1 | ((int16_t)sbus_buf[6] << 7 )) & 0x07FF;
    temp_ch[4] = ((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF;
    temp_ch[5] = ((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 )  | (int16_t)sbus_buf[9] <<  9 ) & 0x07FF;
    temp_ch[6] = ((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF;
    temp_ch[7] = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF;
    
    temp_ch[8] = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
    temp_ch[9] = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
    temp_ch[10] = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 )  | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
    temp_ch[11] = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
    temp_ch[12] = ((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
    temp_ch[13] = ((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 )  | (int16_t)sbus_buf[20] <<  9 ) & 0x07FF;
    temp_ch[14] = ((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
    temp_ch[15] = ((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;
    
    // 数据合法性验证
    uint8_t valid_data = 1;
    for (int i = 0; i < SBUS_RX_BUF_CH_NUM; i++)
    {
        // 应用偏移量
        temp_ch[i] -= REMOTE_CH_VALUE_OFFSET;
        
        // 检查是否在合理范围内 (-1000 到 1000)
        if (temp_ch[i] < -1000 || temp_ch[i] > 1000) 
        {
            valid_data = 0;
            break;
        }
    }
		//关机时
    if( temp_ch[2]==0x0320)
		{
			temp_ch[2]=0x0000;
		}
    // 只有在数据验证通过时才更新通道值
    if (valid_data)
    {
        memcpy(remote_ch_point, temp_ch, sizeof(int16_t) * SBUS_RX_BUF_CH_NUM);
    }
}

/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
static void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i = 0;
    
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

//串口中断
void USART3_IRQHandler(void)
{
    uint32_t isrflags = READ_REG(huart3.Instance->SR);
    uint32_t cr1its = READ_REG(huart3.Instance->CR1);
    
    // 错误处理 (奇偶校验、帧、噪声、溢出)
    if ((isrflags & (UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE)) != 0)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    
    // 接收到数据标志
    if (((isrflags & UART_FLAG_RXNE) != 0) && ((cr1its & USART_CR1_RXNEIE) != 0))
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    
    // 空闲中断处理
    if ((isrflags & UART_FLAG_IDLE) != 0)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前使用内存缓冲区0 */
            
            // 禁用DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // 获取数据长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // 重设数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // 切换到缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            // 重新启用DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            
            // 验证帧并处理
            if (validate_sbus_frame(sbus_rx_buf[0], this_time_rx_len))
            {
                sbus_to_remote_ch(sbus_rx_buf[0], remote_ch);
                // 记录数据接收时间
                detect_hook(DBUS_TOE);
            }
        }
        else
        {
            /* 当前使用内存缓冲区1 */
            
            // 禁用DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // 获取数据长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // 重设数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // 切换到缓冲区0 - 统一使用寄存器方式，保持一致性
            hdma_usart3_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            // 重新启用DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            // 验证帧并处理
            if (validate_sbus_frame(sbus_rx_buf[1], this_time_rx_len))
            {
                sbus_to_remote_ch(sbus_rx_buf[1], remote_ch);
                // 记录数据接收时间
                detect_hook(DBUS_TOE);
            }
        }
    }
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
    memset(remote_ch, 0, sizeof(remote_ch));
}

