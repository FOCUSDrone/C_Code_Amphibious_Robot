/**
 * @file ft_servo_uart.h
 * @brief 飞特串行舵机UART通信层
 * @date 2025.3.6
 * @author 移植自txl
 */

#ifndef _FT_SERVO_UART_H
#define _FT_SERVO_UART_H

#include "stm32f4xx_hal.h"  // 根据DJI C型开发板的STM32型号调整
#include <stdint.h>

/**
 * @brief 初始化串口
 * @param huart 串口句柄
 */
void ft_servo_uart_init(UART_HandleTypeDef *huart);

/**
 * @brief 发送数据
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 成功发送的字节数
 */
int ft_servo_uart_send(uint8_t *data, uint16_t len);

/**
 * @brief 接收数据
 * @param data 数据缓冲区
 * @param len 要接收的字节数
 * @return 成功接收的字节数
 */
int ft_servo_uart_receive(uint8_t *data, uint16_t len);

/**
 * @brief 延时函数
 */
void ft_servo_uart_delay(void);

#endif /* _FT_SERVO_UART_H */
