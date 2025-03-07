/**
 * @file ft_servo_protocol.c
 * @brief 飞特串行舵机通信协议层实现
 * @date 2025.3.6
 * @author 移植自txl
 */

#include "ft_servo_protocol.h"
#include "ft_servo_hw.h"

static uint8_t Level = 1;  // 舵机返回等级1,默认写指令开启应答
static uint8_t End = 0;    // 处理器大小端结构,默认小端存储格式
static uint8_t u8Status;   // 舵机状态
static uint8_t u8Error;    // 通信状态

/**
 * @brief 初始化协议层
 */
void ft_protocol_init(void)
{
    Level = 1;
    End = 0;
    u8Status = 0;
    u8Error = 0;
}

/**
 * @brief 设置大小端模式
 * @param _End 0-小端模式(默认)，1-大端模式
 */
void ft_protocol_set_end(uint8_t _End)
{
    End = _End;
}

/**
 * @brief 获取当前大小端模式
 * @return 当前大小端模式：0-小端，1-大端
 */
uint8_t ft_protocol_get_end(void)
{
    return End;
}

/**
 * @brief 设置应答等级
 * @param _Level 0-无应答，1-有应答(默认)
 */
void ft_protocol_set_level(uint8_t _Level)
{
    Level = _Level;
}

/**
 * @brief 获取舵机状态
 * @return 舵机状态
 */
int ft_protocol_get_state(void)
{
    return u8Status;
}

/**
 * @brief 获取最后一次通信错误
 * @return 错误码
 */
int ft_protocol_get_last_error(void)
{
    return u8Error;
}

/**
 * @brief 1个16位数拆分为2个8位数
 * @param DataL 低位
 * @param DataH 高位
 * @param Data 16位数据
 */
void ft_protocol_host2scs(uint8_t *DataL, uint8_t* DataH, int Data)
{
    if (End) {
        *DataL = (Data >> 8);
        *DataH = (Data & 0xff);
    } else {
        *DataH = (Data >> 8);
        *DataL = (Data & 0xff);
    }
}

/**
 * @brief 2个8位数组合为1个16位数
 * @param DataL 低位
 * @param DataH 高位
 * @return 组合后的16位数据
 */
int ft_protocol_scs2host(uint8_t DataL, uint8_t DataH)
{
    int Data;
    if (End) {
        Data = DataL;
        Data <<= 8;
        Data |= DataH;
    } else {
        Data = DataH;
        Data <<= 8;
        Data |= DataL;
    }
    return Data;
}

/**
 * @brief 写缓冲区
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 数据长度
 * @param Fun 功能码
 */
static void write_buf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
    uint8_t i;
    uint8_t msgLen = 2;
    uint8_t bBuf[6];
    uint8_t CheckSum = 0;
    
    // 帧头固定为0xFF 0xFF
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = ID;
    bBuf[4] = Fun;
    
    if (nDat) {
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        ft_servo_hw_write(bBuf, 6);
    } else {
        bBuf[3] = msgLen;
        ft_servo_hw_write(bBuf, 5);
    }
    
    // 计算校验和
    CheckSum = ID + msgLen + Fun + MemAddr;
    if (nDat) {
        for (i = 0; i < nLen; i++) {
            CheckSum += nDat[i];
        }
        ft_servo_hw_write(nDat, nLen);
    }
    CheckSum = ~CheckSum; // 取反
    ft_servo_hw_write(&CheckSum, 1);
}

/**
 * @brief 检查帧头
 * @return 是否成功：1-成功，0-失败
 */
static int check_head(void)
{
    uint8_t bDat;
    uint8_t bBuf[2] = {0, 0};
    uint8_t Cnt = 0;
    
    while (1) {
        if (!ft_servo_hw_read(&bDat, 1)) {
            return 0;
        }
        bBuf[1] = bBuf[0];
        bBuf[0] = bDat;
        if (bBuf[0] == 0xff && bBuf[1] == 0xff) {
            break;
        }
        Cnt++;
        if (Cnt > 10) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief 指令应答
 * @param ID 舵机ID
 * @return 是否成功：1-成功，0-失败
 */
static int ack(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    
    u8Error = 0;
    if (ID != 0xfe && Level) {
        if (!check_head()) {
            u8Error = FT_ERR_NO_REPLY;
            return 0;
        }
        
        u8Status = 0;
        if (ft_servo_hw_read(bBuf, 4) != 4) {
            u8Error = FT_ERR_NO_REPLY;
            return 0;
        }
        
        if (bBuf[0] != ID) {
            u8Error = FT_ERR_SLAVE_ID;
            return 0;
        }
        
        if (bBuf[1] != 2) {
            u8Error = FT_ERR_BUFF_LEN;
            return 0;
        }
        
        calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
        if (calSum != bBuf[3]) {
            u8Error = FT_ERR_CRC_CMP;
            return 0;            
        }
        
        u8Status = bBuf[2];
    }
    return 1;
}

/**
 * @brief 普通写指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 数据长度
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_gen_write(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    ft_servo_hw_flush_rx();
    write_buf(ID, MemAddr, nDat, nLen, FT_INST_WRITE);
    ft_servo_hw_flush_tx();
    return ack(ID);
}

/**
 * @brief 异步写指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 数据长度
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_reg_write(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    ft_servo_hw_flush_rx();
    write_buf(ID, MemAddr, nDat, nLen, FT_INST_REG_WRITE);
    ft_servo_hw_flush_tx();
    return ack(ID);
}

/**
 * @brief 异步写执行指令
 * @param ID 舵机ID，通常为0xFE(广播ID)
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_reg_action(uint8_t ID)
{
    ft_servo_hw_flush_rx();
    write_buf(ID, 0, NULL, 0, FT_INST_REG_ACTION);
    ft_servo_hw_flush_tx();
    return ack(ID);
}

/**
 * @brief 同步写指令
 * @param ID 舵机ID数组
 * @param IDN 舵机数量
 * @param MemAddr 内存表地址
 * @param nDat 写入数据
 * @param nLen 每个舵机的数据长度
 */
void ft_protocol_sync_write(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    uint8_t mesLen = ((nLen + 1) * IDN + 4);
    uint8_t Sum = 0;
    uint8_t bBuf[7];
    uint8_t i, j;
    
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = 0xfe;  // 广播ID
    bBuf[3] = mesLen;
    bBuf[4] = FT_INST_SYNC_WRITE;
    bBuf[5] = MemAddr;
    bBuf[6] = nLen;
    
    ft_servo_hw_flush_rx();
    ft_servo_hw_write(bBuf, 7);

    // 计算校验和
    Sum = 0xfe + mesLen + FT_INST_SYNC_WRITE + MemAddr + nLen;

    for (i = 0; i < IDN; i++) {
        ft_servo_hw_write(&ID[i], 1);
        ft_servo_hw_write(nDat + i * nLen, nLen);
        Sum += ID[i];
        for (j = 0; j < nLen; j++) {
            Sum += nDat[i * nLen + j];
        }
    }
    Sum = ~Sum; // 取反
    ft_servo_hw_write(&Sum, 1);
    ft_servo_hw_flush_tx();
}

/**
 * @brief 写一个字节
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param bDat 写入数据
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_write_byte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
    ft_servo_hw_flush_rx();
    write_buf(ID, MemAddr, &bDat, 1, FT_INST_WRITE);
    ft_servo_hw_flush_tx();
    return ack(ID);
}

/**
 * @brief 写两个字节(一个字)
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param wDat 写入数据
 * @return 是否成功：1-成功，0-失败
 */
int ft_protocol_write_word(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
    uint8_t buf[2];
    ft_protocol_host2scs(buf + 0, buf + 1, wDat);
    ft_servo_hw_flush_rx();
    write_buf(ID, MemAddr, buf, 2, FT_INST_WRITE);
    ft_servo_hw_flush_tx();
    return ack(ID);
}

/**
 * @brief 读指令
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @param nData 读取数据缓冲区
 * @param nLen 要读取的长度
 * @return 读取的字节数，0表示失败
 */
int ft_protocol_read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen)
{
    int Size;
    uint8_t bBuf[4];
    uint8_t calSum;
    uint8_t i;
    
    ft_servo_hw_flush_rx();
    write_buf(ID, MemAddr, &nLen, 1, FT_INST_READ);
    ft_servo_hw_flush_tx();
    
    u8Error = 0;
    if (!check_head()) {
        u8Error = FT_ERR_NO_REPLY;
        return 0;
    }
    
    if (ft_servo_hw_read(bBuf, 3) != 3) {
        u8Error = FT_ERR_NO_REPLY;
        return 0;
    }
    
    if (bBuf[0] != ID && ID != 0xfe) {
        u8Error = FT_ERR_SLAVE_ID;
        return 0;
    }
    
    if (bBuf[1] != (nLen + 2)) {
        u8Error = FT_ERR_BUFF_LEN;
        return 0;
    }
    
    Size = ft_servo_hw_read(nData, nLen);
    if (Size != nLen) {
        u8Error = FT_ERR_NO_REPLY;
        return 0;
    }
    
    if (ft_servo_hw_read(bBuf + 3, 1) != 1) {
        u8Error = FT_ERR_NO_REPLY;
        return 0;
    }
    
    // 校验和验证
    calSum = bBuf[0] + bBuf[1] + bBuf[2];
    for (i = 0; i < Size; i++) {
        calSum += nData[i];
    }
    calSum = ~calSum;
    if (calSum != bBuf[3]) {
        u8Error = FT_ERR_CRC_CMP;
        return 0;
    }
    
    u8Status = bBuf[2];
    return Size;
}

/**
 * @brief 读一个字节
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @return 读取的数据，-1表示失败
 */
int ft_protocol_read_byte(uint8_t ID, uint8_t MemAddr)
{
    uint8_t bDat;
    int Size = ft_protocol_read(ID, MemAddr, &bDat, 1);
    if (Size != 1) {
        return -1;
    } else {
        return bDat;
    }
}

/**
 * @brief 读两个字节(一个字)
 * @param ID 舵机ID
 * @param MemAddr 内存表地址
 * @return 读取的数据，-1表示失败
 */
int ft_protocol_read_word(uint8_t ID, uint8_t MemAddr)
{    
    uint8_t nDat[2];
    int Size;
    uint16_t wDat;
    
    Size = ft_protocol_read(ID, MemAddr, nDat, 2);
    if (Size != 2)
        return -1;
    
    wDat = ft_protocol_scs2host(nDat[0], nDat[1]);
    return wDat;
}

/**
 * @brief PING指令
 * @param ID 舵机ID
 * @return 舵机ID，-1表示失败
 */
int ft_protocol_ping(uint8_t ID)
{
    uint8_t bBuf[4];
    uint8_t calSum;
    
    ft_servo_hw_flush_rx();
    write_buf(ID, 0, NULL, 0, FT_INST_PING);
    ft_servo_hw_flush_tx();
    
    u8Status = 0;
    if (!check_head()) {
        u8Error = FT_ERR_NO_REPLY;
        return -1;
    }
    
    u8Error = 0;
    if (ft_servo_hw_read(bBuf, 4) != 4) {
        u8Error = FT_ERR_NO_REPLY;
        return -1;
    }
    
    if (bBuf[0] != ID && ID != 0xfe) {
        u8Error = FT_ERR_SLAVE_ID;
        return -1;
    }
    
    if (bBuf[1] != 2) {
        u8Error = FT_ERR_BUFF_LEN;
        return -1;
    }
    
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3]) {
        u8Error = FT_ERR_CRC_CMP;
        return -1;            
    }
    
    u8Status = bBuf[2];
    return bBuf[0];
}
