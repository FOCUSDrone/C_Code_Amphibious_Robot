/**
  ******************************************************************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025.2.15       tanjiong        1. ���
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

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_remote_ch(volatile const uint8_t *sbus_buf, int16_t *remote_ch_point);


//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            
            if(this_time_rx_len == AT9S_FRAME_LENGTH)
            {
                sbus_to_remote_ch(sbus_rx_buf[0], remote_ch);
                //��¼���ݽ���ʱ��
                detect_hook(DBUS_TOE);
//                sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == AT9S_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_remote_ch(sbus_rx_buf[1], remote_ch);
                //��¼���ݽ���ʱ��
                detect_hook(DBUS_TOE);
//                sbus_to_usart1(sbus_rx_buf[1]);
            }
        }
    }

}


/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     remote_ch_point: ң��������ָ
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
  * @brief          ͨ��usart1����sbus����,��usart3_IRQHandle����
  * @param[in]      sbus: sbus����, 18�ֽ�
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
  * @brief          ���ң����ͨ������ָ��
  * @param[in]      none
  * @retval         ���ݵ�ַ
  */
const int16_t *get_remote_ch_point(void)
{
    return remote_ch;
}


/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void remote_receive_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    memset(remote_ch, 0, SBUS_RX_BUF_NUM);
}

