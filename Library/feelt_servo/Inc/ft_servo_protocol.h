/**
 * @file ft_servo_protocol.h
 * @brief 飞特串行舵机通信协议层
 * @date 2025.3.6
 * @author 移植自txl
 */

#ifndef _FT_SERVO_PROTOCOL_H
#define _FT_SERVO_PROTOCOL_H

#include <stdint.h>

// 指令定义
#define FT_INST_PING 0x01       // PING指令
#define FT_INST_READ 0x02       // 读指令
#define FT_INST_WRITE 0x03      // 写指令
#define FT_INST_REG_WRITE 0x04  // 异步写指令
#define FT_INST_REG_ACTION 0x05 // 异步写执行指令
#define FT_INST_SYNC_WRITE 0x83 // 同步写指令

// 错误码定义
enum FT_ERR_LIST
{
    FT_ERR_NO_REPLY = 1, // 无应答
    FT_ERR_CRC_CMP  = 2, // CRC校验错误
    FT_ERR_SLAVE_ID = 3, // 从机ID错误
    FT_ERR_BUFF_LEN = 4, // 缓冲区长度错误
};

/**
 * @brief 初始化协议层
 */
void ft_protocol_init(void);

/**
 * @brief 设置大小端模式
 * @param _End 0-小端模式(默认)，1-大端模式
 */
void ft_protocol_set_end(uint8_t _End);

/**
 * @brief 获取当前大小端模式
 * @return 当前大小端模式：0-小端，1-大端
 */
uint8_t ft_protocol_get_end(void);

/**
 * @brief 设置应答等级
 * @param _Level 0-无应答，1-有应答(默认)
 */
void ft_protocol_set_level(uint8_t _Level);

/**
 * @brief 获取舵机状态
 * @return 舵机状态
 */
int ft_protocol_get_state(void);

/**
 * @brief 获取最后一次通信错误
 * @return 错误码
 */
int ft_protocol_get_last_error(void);

/**
 * @brief 普通写指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 数据长度
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_gen_write(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

/**
 * @brief 异步写指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 数据长度
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_reg_write(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

/**
 * @brief 异步写执行指令
 * @param ID 舵机ID，通常为0xFE(广播ID)
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_reg_action(uint8_t ID);

/**
 * @brief 同步写指令
 * @param ID 舵机ID数组
 * @param IDN 舵机数量
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 每个舵机的数据长度
 */
void ft_protocol_sync_write(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);

/**
 * @brief 写一个字节
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param bDat 写入数据
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_write_byte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);

/**
 * @brief 写两个字节(一个字)
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param wDat 写入数据
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_write_word(uint8_t ID, uint8_t MemAddr, uint16_t wDat);

/**
 * @brief 读指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nData 读取数据缓冲区
 * @param nLen 要读取的长度
 * @return 读取的字节数，0表示失败
 */
int ft_protocol_read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);

/**
 * @brief 读一个字节
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @return 读取的数据，-1表示失败
 */
int ft_protocol_read_byte(uint8_t ID, uint8_t MemAddr);

/**
 * @brief 读两个字节(一个字)
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @return 读取的数据，-1表示失败
 */
int ft_protocol_read_word(uint8_t ID, uint8_t MemAddr);

/**
 * @brief PING指令
 * @param ID 舵机ID
 * @return 舵机ID，-1表示失败
 */
int ft_protocol_ping(uint8_t ID);

/**
 * @brief 1个16位数拆分为2个8位数
 * @param DataL 低位
 * @param DataH 高位
 * @param Data 16位数据
 */
void ft_protocol_host2scs(uint8_t *DataL, uint8_t* DataH, int Data);

/**
 * @brief 2个8位数组合为1个16位数
 * @param DataL 低位
 * @param DataH 高位
 * @return 组合后的16位数据
 */
int ft_protocol_scs2host(uint8_t DataL, uint8_t DataH);

#endif /* _FT_SERVO_PROTOCOL_H */