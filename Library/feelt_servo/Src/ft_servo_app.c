/**
 * @file ft_servo_app.c
 * @brief 飞特SMS系列串行舵机应用层实现
 * @date 2025.3.6
 * @author 移植自txl
 */

#include "ft_servo_app.h"
#include "ft_servo_hw.h"
#include "ft_servo_protocol.h"
#include <string.h>

// 缓存所有反馈数据的缓冲区
static uint8_t Mem[FT_SMS_PRESENT_CURRENT_H - FT_SMS_PRESENT_POSITION_L + 1];

/**
 * @brief 初始化应用层
 * @param huart 串口句柄
 */
void ft_servo_app_init(UART_HandleTypeDef *huart)
{
    ft_servo_hw_init(huart);
    ft_protocol_init();
    memset(Mem, 0, sizeof(Mem));
}

/**
 * @brief 普通写位置指令
 * @param ID 舵机ID
 * @param Position 目标位置
 * @param Speed 速度限制
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[7];
    
    // 处理负位置值
    if (Position < 0) {
        Position = -Position;
        Position |= (1 << 15); // 设置最高位为符号位
    }

    bBuf[0] = ACC; // 加速度
    ft_protocol_host2scs(bBuf + 1, bBuf + 2, Position); // 目标位置
    ft_protocol_host2scs(bBuf + 3, bBuf + 4, 0); // 运行时间，SMS不使用
    ft_protocol_host2scs(bBuf + 5, bBuf + 6, Speed); // 速度限制
    
    return ft_protocol_gen_write(ID, FT_SMS_ACC, bBuf, 7);
}

/**
 * @brief 异步写位置指令
 * @param ID 舵机ID
 * @param Position 目标位置
 * @param Speed 速度限制
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_reg_write_pos(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[7];
    
    // 处理负位置值
    if (Position < 0) {
        Position = -Position;
        Position |= (1 << 15); // 设置最高位为符号位
    }

    bBuf[0] = ACC; // 加速度
    ft_protocol_host2scs(bBuf + 1, bBuf + 2, Position); // 目标位置
    ft_protocol_host2scs(bBuf + 3, bBuf + 4, 0); // 运行时间，SMS不使用
    ft_protocol_host2scs(bBuf + 5, bBuf + 6, Speed); // 速度限制
    
    return ft_protocol_reg_write(ID, FT_SMS_ACC, bBuf, 7);
}

/**
 * @brief 异步写位置执行
 */
void ft_servo_app_reg_write_action(void)
{
    ft_protocol_reg_action(0xfe); // 广播指令
}

/**
 * @brief 同步写位置指令
 * @param ID 舵机ID数组
 * @param IDN 舵机数量
 * @param Position 目标位置数组
 * @param Speed 速度限制数组，可为NULL
 * @param ACC 加速度数组，可为NULL
 */
void ft_servo_app_sync_write_pos(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
    uint8_t offbuf[32 * 7]; // 最多支持32个舵机
    uint8_t i;
    uint16_t V;
    
    for (i = 0; i < IDN; i++) {
        // 处理负位置值
        if (Position[i] < 0) {
            Position[i] = -Position[i];
            Position[i] |= (1 << 15); // 设置最高位为符号位
        }

        // 如果没提供速度，则默认为0
        if (Speed) {
            V = Speed[i];
        } else {
            V = 0;
        }
        
        // 如果没提供加速度，则默认为0
        if (ACC) {
            offbuf[i * 7] = ACC[i];
        } else {
            offbuf[i * 7] = 0;
        }
        
        ft_protocol_host2scs(offbuf + i * 7 + 1, offbuf + i * 7 + 2, Position[i]); // 目标位置
        ft_protocol_host2scs(offbuf + i * 7 + 3, offbuf + i * 7 + 4, 0); // 运行时间，SMS不使用
        ft_protocol_host2scs(offbuf + i * 7 + 5, offbuf + i * 7 + 6, V); // 速度限制
    }
    
    ft_protocol_sync_write(ID, IDN, FT_SMS_ACC, offbuf, 7);
}

/**
 * @brief 恒速模式设置
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_wheel_mode(uint8_t ID)
{
    return ft_protocol_write_byte(ID, FT_SMS_MODE, 1);        
}

/**
 * @brief 恒速模式控制指令
 * @param ID 舵机ID
 * @param Speed 目标速度
 * @param ACC 加速度
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_write_speed(uint8_t ID, int16_t Speed, uint8_t ACC)
{
    uint8_t bBuf[2];
    
    // 处理负速度值
    if (Speed < 0) {
        Speed = -Speed;
        Speed |= (1 << 15); // 设置最高位为符号位
    }
    
    // 先设置加速度
    bBuf[0] = ACC;
    ft_protocol_gen_write(ID, FT_SMS_ACC, bBuf, 1);
    
    // 再设置速度
    ft_protocol_host2scs(bBuf + 0, bBuf + 1, Speed);
    ft_protocol_gen_write(ID, FT_SMS_GOAL_SPEED_L, bBuf, 2);
    
    return 1;
}

/**
 * @brief 中位校准
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_calibration_ofs(uint8_t ID)
{
    return ft_protocol_write_byte(ID, FT_SMS_TORQUE_ENABLE, 128);
}

/**
 * @brief 扭矩使能
 * @param ID 舵机ID
 * @param Enable 0-关闭，1-开启
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_torque_enable(uint8_t ID, uint8_t Enable)
{
    return ft_protocol_write_byte(ID, FT_SMS_TORQUE_ENABLE, Enable);
}

/**
 * @brief 解锁EPROM
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_unlock_eprom(uint8_t ID)
{
    return ft_protocol_write_byte(ID, FT_SMS_LOCK, 0);
}

/**
 * @brief 锁定EPROM
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
int ft_servo_app_lock_eprom(uint8_t ID)
{
    return ft_protocol_write_byte(ID, FT_SMS_LOCK, 1);
}

/**
 * @brief 批量读取舵机反馈信息
 * @param ID 舵机ID
 * @return 读取的字节数，-1表示失败
 */
int ft_servo_app_feedback(uint8_t ID)
{
    int nLen = ft_protocol_read(ID, FT_SMS_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if (nLen != sizeof(Mem)) {
        return -1;
    }
    return nLen;
}

/**
 * @brief 读位置
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机位置，-1表示失败
 */
int ft_servo_app_read_pos(uint8_t ID)
{
    int Pos;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Pos = Mem[FT_SMS_PRESENT_POSITION_H - FT_SMS_PRESENT_POSITION_L];
        Pos <<= 8;
        Pos |= Mem[FT_SMS_PRESENT_POSITION_L - FT_SMS_PRESENT_POSITION_L];
    } else {
        // 直接读取舵机
        Pos = ft_protocol_read_word(ID, FT_SMS_PRESENT_POSITION_L);
        if (Pos == -1) return -1;
    }
    
    // 处理负位置值
    if (Pos & (1 << 15)) {
        Pos = -(Pos & ~(1 << 15));
    }
    
    return Pos;
}

/**
 * @brief 读速度
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机速度，-1表示失败
 */
int ft_servo_app_read_speed(uint8_t ID)
{
    int Speed;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Speed = Mem[FT_SMS_PRESENT_SPEED_H - FT_SMS_PRESENT_POSITION_L];
        Speed <<= 8;
        Speed |= Mem[FT_SMS_PRESENT_SPEED_L - FT_SMS_PRESENT_POSITION_L];
    } else {
        // 直接读取舵机
        Speed = ft_protocol_read_word(ID, FT_SMS_PRESENT_SPEED_L);
        if (Speed == -1) return -1;
    }
    
    // 处理负速度值
    if (Speed & (1 << 15)) {
        Speed = -(Speed & ~(1 << 15));
    }
    
    return Speed;
}

/**
 * @brief 读输出扭力
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机负载，-1表示失败
 */
int ft_servo_app_read_load(uint8_t ID)
{
    int Load;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Load = Mem[FT_SMS_PRESENT_LOAD_H - FT_SMS_PRESENT_POSITION_L];
        Load <<= 8;
        Load |= Mem[FT_SMS_PRESENT_LOAD_L - FT_SMS_PRESENT_POSITION_L];
    } else {
        // 直接读取舵机
        Load = ft_protocol_read_word(ID, FT_SMS_PRESENT_LOAD_L);
        if (Load == -1) return -1;
    }
    
    // 处理负负载值，注意SMS舵机负载符号位在第10位
    if (Load & (1 << 10)) {
        Load = -(Load & ~(1 << 10));
    }
    
    return Load;
}

/**
 * @brief 读电压
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机电压，-1表示失败
 */
int ft_servo_app_read_voltage(uint8_t ID)
{
    int Voltage;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Voltage = Mem[FT_SMS_PRESENT_VOLTAGE - FT_SMS_PRESENT_POSITION_L];    
    } else {
        // 直接读取舵机
        Voltage = ft_protocol_read_byte(ID, FT_SMS_PRESENT_VOLTAGE);
    }
    
    return Voltage;
}

/**
 * @brief 读温度
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机温度，-1表示失败
 */
int ft_servo_app_read_temperature(uint8_t ID)
{
    int Temper;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Temper = Mem[FT_SMS_PRESENT_TEMPERATURE - FT_SMS_PRESENT_POSITION_L];    
    } else {
        // 直接读取舵机
        Temper = ft_protocol_read_byte(ID, FT_SMS_PRESENT_TEMPERATURE);
    }
    
    return Temper;
}

/**
 * @brief 读移动状态
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机移动状态，-1表示失败
 */
int ft_servo_app_read_move(uint8_t ID)
{
    int Move;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Move = Mem[FT_SMS_MOVING - FT_SMS_PRESENT_POSITION_L];    
    } else {
        // 直接读取舵机
        Move = ft_protocol_read_byte(ID, FT_SMS_MOVING);
    }
    
    return Move;
}

/**
 * @brief 读电流
 * @param ID 舵机ID，0xFF表示使用缓存数据
 * @return 舵机电流，-1表示失败
 */
int ft_servo_app_read_current(uint8_t ID)
{
    int Current;
    
    if (ID == 0xFF) {
        // 使用缓存数据
        Current = Mem[FT_SMS_PRESENT_CURRENT_H - FT_SMS_PRESENT_POSITION_L];
        Current <<= 8;
        Current |= Mem[FT_SMS_PRESENT_CURRENT_L - FT_SMS_PRESENT_POSITION_L];
    } else {
        // 直接读取舵机
        Current = ft_protocol_read_word(ID, FT_SMS_PRESENT_CURRENT_L);
        if (Current == -1) return -1;
    }
    
    // 处理负电流值
    if (Current & (1 << 15)) {
        Current = -(Current & ~(1 << 15));
    }
    
    return Current;
}