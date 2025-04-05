/**
  ****************************(C) COPYRIGHT 2025 Your Company****************************
  * @file       bsp_i2c.h
  * @brief      I2C底层通信接口，提供通用I2C读写功能
  * @note       这是BSP层代码，与具体传感器无关，提供通用I2C通信功能
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-22-2025     Feiziben       1. 创建文件
  *
  @verbatim
  ==============================================================================
  该文件提供通用I2C通信接口，包括：
  1. I2C初始化
  2. I2C单字节读写
  3. I2C多字节读写
  4. I2C总线错误处理
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Your Company****************************
  */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/* 通用I2C通信超时时间(ms) */
#define I2C_TIMEOUT_MS 10

/**
  * @brief          I2C初始化
  * @param[in]      hi2c: I2C句柄指针
  * @retval         0: 初始化成功; 非0: 失败
  */
extern uint8_t bsp_i2c_init(I2C_HandleTypeDef *hi2c);

/**
  * @brief          从I2C设备读取单个寄存器
  * @param[in]      hi2c: I2C句柄指针
  * @param[in]      device_address: 设备地址(7位地址)
  * @param[in]      reg_address: 寄存器地址
  * @param[out]     p_data: 数据存储指针
  * @retval         0: 读取成功; 非0: 失败
  */
extern uint8_t bsp_i2c_read_reg(I2C_HandleTypeDef *hi2c, uint8_t device_address, 
                                uint8_t reg_address, uint8_t *p_data);

/**
  * @brief          从I2C设备读取多个寄存器
  * @param[in]      hi2c: I2C句柄指针
  * @param[in]      device_address: 设备地址(7位地址)
  * @param[in]      reg_address: 寄存器起始地址
  * @param[out]     p_data: 数据存储指针
  * @param[in]      len: 要读取的字节数
  * @retval         0: 读取成功; 非0: 失败
  */
extern uint8_t bsp_i2c_read_multi_reg(I2C_HandleTypeDef *hi2c, uint8_t device_address, 
                                     uint8_t reg_address, uint8_t *p_data, uint8_t len);

/**
  * @brief          向I2C设备写入单个寄存器
  * @param[in]      hi2c: I2C句柄指针
  * @param[in]      device_address: 设备地址(7位地址)
  * @param[in]      reg_address: 寄存器地址
  * @param[in]      data: 要写入的数据
  * @retval         0: 写入成功; 非0: 失败
  */
extern uint8_t bsp_i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t device_address, 
                                uint8_t reg_address, uint8_t data);

/**
  * @brief          向I2C设备写入多个寄存器
  * @param[in]      hi2c: I2C句柄指针
  * @param[in]      device_address: 设备地址(7位地址)
  * @param[in]      reg_address: 寄存器起始地址
  * @param[in]      p_data: 要写入的数据指针
  * @param[in]      len: 要写入的字节数
  * @retval         0: 写入成功; 非0: 失败
  */
extern uint8_t bsp_i2c_write_multi_reg(I2C_HandleTypeDef *hi2c, uint8_t device_address, 
                                      uint8_t reg_address, uint8_t *p_data, uint8_t len);

/**
  * @brief          检查I2C设备是否存在
  * @param[in]      hi2c: I2C句柄指针
  * @param[in]      device_address: 设备地址(7位地址)
  * @retval         0: 设备存在; 非0: 设备不存在
  */
extern uint8_t bsp_i2c_check_device(I2C_HandleTypeDef *hi2c, uint8_t device_address);

#endif /* BSP_I2C_H */

