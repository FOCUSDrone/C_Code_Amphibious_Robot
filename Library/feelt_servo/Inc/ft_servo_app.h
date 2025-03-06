/**
 * @file ft_servo_app.h
 * @brief 飞特SMS系列串行舵机应用层
 * @date 2025.3.6
 * @author 移植自txl
 */

#ifndef _FT_SERVO_APP_H
#define _FT_SERVO_APP_H

#include <stdint.h>
#include "ft_servo_protocol.h"
#include "stm32f4xx_hal.h"  // 根据DJI C型开发板的STM32型号调整

// 波特率定义
#define FT_SMS_1M 0
#define FT_SMS_0_5M 1
#define FT_SMS_250K 2
#define FT_SMS_128K 3
#define FT_SMS_115200 4
#define FT_SMS_76800 5
#define FT_SMS_57600 6
#define FT_SMS_38400 7

// 内存表定义 - EPROM(只读)
#define FT_SMS_MODEL_L 3
#define FT_SMS_MODEL_H 4

// 内存表定义 - EPROM(读写)
#define FT_SMS_ID 5
#define FT_SMS_BAUD_RATE 6
#define FT_SMS_MIN_ANGLE_LIMIT_L 9
#define FT_SMS_MIN_ANGLE_LIMIT_H 10
#define FT_SMS_MAX_ANGLE_LIMIT_L 11
#define FT_SMS_MAX_ANGLE_LIMIT_H 12
#define FT_SMS_CW_DEAD 26
#define FT_SMS_CCW_DEAD 27
#define FT_SMS_OFS_L 31
#define FT_SMS_OFS_H 32
#define FT_SMS_MODE 33

// 内存表定义 - SRAM(读写)
#define FT_SMS_TORQUE_ENABLE 40
#define FT_SMS_ACC 41
#define FT_SMS_GOAL_POSITION_L 42
#define FT_SMS_GOAL_POSITION_H 43
#define FT_SMS_GOAL_TIME_L 44
#define FT_SMS_GOAL_TIME_H 45
#define FT_SMS_GOAL_SPEED_L 46
#define FT_SMS_GOAL_SPEED_H 47
#define FT_SMS_LOCK 55

// 内存表定义 - SRAM(只读)
#define FT_SMS_PRESENT_POSITION_L 56
#define FT_SMS_PRESENT_POSITION_H 57
#define FT_SMS_PRESENT_SPEED_L 58
#define FT_SMS_PRESENT_SPEED_H 59
#define FT_SMS_PRESENT_LOAD_L 60
#define FT_SMS_PRESENT_LOAD_H 61
#define FT_SMS_PRESENT_VOLTAGE 62
#define FT_SMS_PRESENT_TEMPERATURE 63
#define FT_SMS_MOVING 66
#define FT_SMS_PRESENT_CURRENT_L 69
#define FT_SMS_PRESENT_CURRENT_H 70

/**
 * @brief 初始化应用层
 * @param huart 串口句柄
 */
void ft_servo_app_init(UART_HandleTypeDef *huart);

/**
 * @brief 普通写位置指令
 * @param ID 舵机ID
 * @param Position 目标位置
 * @param Speed 速度限制
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);

/**
 * @brief 异步写位置指令
 * @param ID 舵机ID
 * @param Position 目标位置
 * @param Speed 速度限制
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_reg_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);

/**
 * @brief 异步写位置执行
 */
void ft_servo_app_reg_write_action(void);

/**
 * @brief 同步写位置指令
 * @param ID 舵机ID数组
 * @param IDN 舵机数量
 * @param Position 目标位置数组
 * @param Speed 速度限制数组，可为NULL
 * @param ACC 加速度数组，可为NULL
 */
void ft_servo_app_sync_write_pos(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);

/**
 * @brief 恒速模式设置
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_wheel_mode(uint8_t ID);

/**
 * @brief 恒速模式控制指令
 * @param ID 舵机ID
 * @param Speed 目标速度
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_write_speed(uint8_t ID, int16_t Speed, uint8_t ACC);

/**
 * @brief 中位校准
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_calibration_ofs(uint8_t ID);

/**
 * @brief 扭矩使能
 * @param ID 舵机ID
 * @param Enable 0-关闭，1-开启
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_torque_enable(uint8_t ID, uint8_t Enable);

/**
 * @brief 解锁EPROM
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_unlock_eprom(uint8_t ID);

/**
 * @brief 锁定EPROM
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_lock_eprom(uint8_t ID);

/**
 * @brief 批量读取舵机反馈信息
 * @param ID 舵机ID
 * @return 读取的字节数，-1表示失败
 */
int ft_servo_app_feedback(uint8_t ID);

/**
 * @brief 读位置
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机位置，-1表示失败
 */
int ft_servo_app_read_pos(uint8_t ID);

/**
 * @brief 读速度
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机速度，-1表示失败
 */
int ft_servo_app_read_speed(uint8_t ID);

/**
 * @brief 读输出扭力
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机负载，-1表示失败
 */
int ft_servo_app_read_load(uint8_t ID);

/**
 * @brief 读电压
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机电压，-1表示失败
 */
int ft_servo_app_read_voltage(uint8_t ID);

/**
 * @brief 读温度
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机温度，-1表示失败
 */
int ft_servo_app_read_temperature(uint8_t ID);

/**
 * @brief 读移动状态
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机移动状态，-1表示失败
 */
int ft_servo_app_read_move(uint8_t ID);

/**
 * @brief 读电流
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机电流，-1表示失败
 */
int ft_servo_app_read_current(uint8_t ID);

#endif /* _FT_SERVO_APP_H */