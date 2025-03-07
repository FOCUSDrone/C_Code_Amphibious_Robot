/**
 * @file ft_servo_uart.c
 * @brief 飞特串行舵机UART通信层实现
 * @date 2025.3.6
 * @author 移植自txl
 */

#include "ft_servo_uart.h"

static UART_HandleTypeDef *servo_uart; // 舵机使用的串口句柄

/**
 * @brief 初始化串口
 * @param huart 串口句柄
 */
void ft_servo_uart_init(UART_HandleTypeDef *huart)
{
    servo_uart = huart;
    
    // 设置串口参数，适用于SMS系列舵机的默认波特率115200
    servo_uart->Init.BaudRate = 115200;
    servo_uart->Init.WordLength = UART_WORDLENGTH_8B;
    servo_uart->Init.StopBits = UART_STOPBITS_1;
    servo_uart->Init.Parity = UART_PARITY_NONE;
    servo_uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    servo_uart->Init.Mode = UART_MODE_TX_RX;
    
    HAL_UART_Init(servo_uart);
}

/**
 * @brief 发送数据
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 成功发送的字节数
 */
int ft_servo_uart_send(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(servo_uart, data, len, 100); // 超时设为100ms
    return (status == HAL_OK) ? len : 0;
}

/**
 * @brief 接收数据
 * @param data 数据缓冲区
 * @param len 要接收的字节数
 * @return 成功接收的字节数
 */
int ft_servo_uart_receive(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(servo_uart, data, len, 100); // 超时设为100ms
    return (status == HAL_OK) ? len : 0;
}

/**
 * @brief 延时函数
 */
void ft_servo_uart_delay(void)
{
    HAL_Delay(1); // 1ms延时，根据实际通信速率可调整
}