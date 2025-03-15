/*
 * SCServo.c
 * 飞特舵机硬件接口层程序
 * 日期: 2024.12.2
 * 作者: txl
 */
#include <stdint.h>
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

uint8_t wBuf[128];
uint8_t wLen = 0;

//FT舵机串口指令发送函数
void ftUart_Send(uint8_t *nDat , int nLen)
{
//    usart1_tx_dma_enable(nDat, nLen);
	HAL_UART_Transmit(&huart1, nDat, nLen, 100);
}

//FT舵机串口指令应答接收函数
int ftUart_Read(uint8_t *nDat, int nLen)
{
	if(HAL_OK!=HAL_UART_Receive(&huart1, nDat, nLen, 100)){
		return 0;
	}else{
		return nLen;
	}
}

//FT舵机总线切换延时，时间大于10us
void ftBus_Delay(void)
{
    osDelay(1);
//	HAL_Delay(1);
}

//UART 接收数据接口
int readSCS(unsigned char *nDat, int nLen)
{
	return ftUart_Read(nDat, nLen);
}

//UART 发送数据接口
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

//接收缓冲区刷新
void rFlushSCS()
{
	ftBus_Delay();
}

//发送缓冲区刷新
void wFlushSCS()
{
	if(wLen){
		ftUart_Send(wBuf, wLen);
		wLen = 0;
	}
}
