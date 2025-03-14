/**
 * @file ft_servo_hw.c
 * @brief 飞特串行舵机硬件抽象层实现
 * @date 2025.3.6
 * @author 移植自txl
 */

#include "ft_servo_hw.h"

static uint8_t tx_buffer[128]; // 发送缓冲区
static uint16_t tx_len = 0;    // 发送缓冲区中的数据长度

/**
 * @brief 初始化硬件
 * @param huart 串口句柄
 */
void ft_servo_hw_init(UART_HandleTypeDef *huart)
{
    ft_servo_uart_init(huart);
    tx_len = 0;
}

/**
 * @brief 写数据到缓冲区
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 成功写入的字节数
 */
int ft_servo_hw_write(uint8_t *data, uint16_t len)
{
    while (len--) {
        if (tx_len < sizeof(tx_buffer)) {
            tx_buffer[tx_len++] = *data++;
        }
    }
    return tx_len;
}

/**
 * @brief 读取数据
 * @param data 数据缓冲区
 * @param len 要读取的字节数
 * @return 成功读取的字节数
 */
int ft_servo_hw_read(uint8_t *data, uint16_t len)
{
    return ft_servo_uart_receive(data, len);
}

/**
 * @brief 刷新接收缓冲区
 */
void ft_servo_hw_flush_rx(void)
{
    ft_servo_uart_delay(); // 短暂延时确保接收缓冲区清空
}

/**
 * @brief 刷新发送缓冲区，实际发送数据
 */
void ft_servo_hw_flush_tx(void)
{
    if (tx_len) {
        ft_servo_uart_send(tx_buffer, tx_len);
        tx_len = 0; // 清空缓冲区
    }
}
