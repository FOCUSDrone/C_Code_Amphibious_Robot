/**
 * @file ft_servo_hw.h
 * @brief 飞特串行舵机硬件抽象层
 * @date 2025.3.6
 * @author 移植自txl
 */

#ifndef _FT_SERVO_HW_H
#define _FT_SERVO_HW_H

#include <stdint.h>
#include "ft_servo_uart.h"

/**
 * @brief 初始化硬件
 * @param huart 串口句柄
 */
void ft_servo_hw_init(UART_HandleTypeDef *huart);

/**
 * @brief 写数据到缓冲区
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 成功写入的字节数
 */
int ft_servo_hw_write(uint8_t *data, uint16_t len);

/**
 * @brief 读取数据
 * @param data 数据缓冲区
 * @param len 要读取的字节数
 * @return 成功读取的字节数
 */
int ft_servo_hw_read(uint8_t *data, uint16_t len);

/**
 * @brief 刷新接收缓冲区
 */
void ft_servo_hw_flush_rx(void);

/**
 * @brief 刷新发送缓冲区，实际发送数据
 */
void ft_servo_hw_flush_tx(void);

#endif /* _FT_SERVO_HW_H */