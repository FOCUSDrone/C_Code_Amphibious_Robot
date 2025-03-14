/*
 * SCServo.c
 * ���ض��Ӳ���ӿڲ����
 * ����: 2024.12.2
 * ����: txl
 */
#include <stdint.h>
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

uint8_t wBuf[128];
uint8_t wLen = 0;

//FT�������ָ��ͺ���
void ftUart_Send(uint8_t *nDat , int nLen)
{
//    usart1_tx_dma_enable(nDat, nLen);
	HAL_UART_Transmit(&huart1, nDat, nLen, 100);
}

//FT�������ָ��Ӧ����պ���
int ftUart_Read(uint8_t *nDat, int nLen)
{
	if(HAL_OK!=HAL_UART_Receive(&huart1, nDat, nLen, 100)){
		return 0;
	}else{
		return nLen;
	}
}

//FT��������л���ʱ��ʱ�����10us
void ftBus_Delay(void)
{
    osDelay(1);
//	HAL_Delay(1);
}

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
	return ftUart_Read(nDat, nLen);
}

//UART �������ݽӿ�
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

//���ջ�����ˢ��
void rFlushSCS()
{
	ftBus_Delay();
}

//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		ftUart_Send(wBuf, wLen);
		wLen = 0;
	}
}
