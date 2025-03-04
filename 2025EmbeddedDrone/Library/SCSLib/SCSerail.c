/*
 * SCServo.c
 * ���ض��Ӳ���ӿڲ����
 * ����: 2024.12.2
 * ����: txl
 */
#include <stdint.h>
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"

uint8_t wBuf[128];
uint8_t wLen = 0;

void ftUart_Send(uint8_t *nDat, int nLen);
int ftUart_Read(uint8_t *nDat, int nLen);
void ftBus_Delay(void);

extern uint8_t **usart1_rx_buf;

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
    memcpy(nDat, usart1_rx_buf[0], nLen);
    return nLen;
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
    vTaskDelay(1);
}

//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		usart1_tx_dma_enable(wBuf, wLen);
		wLen = 0;
	}
}
