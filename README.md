# !这个branch添加了完全未测试的car_bsp,fan_bsp,两个task,Adhesion_task与car_task!并且library中存在经过我完全移植的官方舵机库
# The_C_board_code_for_the_suction_type_amphibious_robot.

The integrated version of the DJI C model development board code for the suction-type amphibious robot.
## 陆空两栖无人机的C板底层控制代码，包括：
- Fleet_svero_controller(uart)
- SUBS_receiver_signal_decoding(subs)
- Suction_fan_controller(pwm)
- TMotor_drone_motors_controller(pwm)
## 开发环境：keilmdk+cubemx
- 利用ARM编译器编译
- PS：千万不要用gcc交叉编译，它与ARM编译器的语法有不同之处，即使cmake文件写对，也要做大量的修改工作（亲身体验，改了一个月改不完）
## [DJI官方例程](https://github.com/RoboMaster/Development-Board-C-Examples)
- very important 学习资料，此处很多代码都参考了。
## [DJI C板教程](https://github.com/FOCUSDrone/C_code_amphibious_robot/blob/main/RoboMaster%E5%BC%80%E5%8F%91%E6%9D%BFC%E5%9E%8B%E5%B5%8C%E5%85%A5%E5%BC%8F%E8%BD%AF%E4%BB%B6%E6%95%99%E7%A8%8B%E6%96%87%E6%A1%A3.pdf)
## [FREETOS快速上手教程](https://github.com/FOCUSDrone/C_code_amphibious_robot/blob/main/FreeRTOS%E5%AE%8C%E5%85%A8%E5%BC%80%E5%8F%91%E6%89%8B%E5%86%8C%E4%B9%8B%E4%B8%8A%E5%86%8C_%E5%BF%AB%E9%80%9F%E5%85%A5%E9%97%A8.pdf)


## 代码架构讲解
- ├── Application/                # 应用层，所有的线程（包括中断）都在这里
- │   ├── Inc/                    # 存放头文件
- │   ├── Src/                    # 存放源文件
- │   │   ├── buzzer_task.c       # 蜂鸣器线程，根据自检线程反映数据，实时报警
- │   │   ├── CAN_receive.c       # CAN总线接收线程
- │   │   ├── detect_task.c       # 自检线程线程，检测各个模块运行状态
- │   │   ├── INS_task.c          # 惯性导航系统线程，运行姿态融合解算算法
- │   │   ├── key_task.c          # 按键线程，debug用途
- │   │   ├── led_task.c          # LED灯线程，三色光流动效果，可以反应系统是否死机
- │   │   ├── remote_receive.c    # 遥控器接收线程，串口usart3中断
- │   │   ├── servo_receive.c     # 舵机接收线程，串口usart1中断
- │   │   ├── test_task.c         # 测试线程，dubug用途
- │   │   └── usb_task.c          # USB虚拟串口与上位机通信线程
- ├── BSP/                        # DJI的C板底层库，是对Core底层的用户封装，构建应用层，导入库需要用此处的函数重构
- ├── Core/                       # CubeMX生成文件，包括main和硬件底层，这里的文件都不需要修改
- ├── Devices/                    # 一些复杂设备的驱动，主要是陀螺仪和压力计芯片的驱动（操作其他芯片都放这里）
- ├── Drivers/                    # CubeMX生成文件，包括freertos操作系统驱动，CMSIS驱动，不需要修改
- ├── Library/                    # 第三方库和自定义算法（包括舵机、数学库）
- ├── MDK-ARM/                    # CubeMX生成文件, IDE配置文件夹，用于keil的编译和储存编译后用于烧录的文件不需要修改
- ├── Middlewares/                # CubeMX生成文件, 中间软件，包括freetos、数学库（CMSIS，相当底层芯片的运算加速）
- └── USB_DEVICE/                 # CubeMX生成文件, 和虚拟串口有关的库，不需要修改
## 特点
飞特SMS系列舵机驱动库
概述
本库为基于STM32的DJI C型开发板提供了飞特SMS系列串行舵机的驱动程序。库采用分层架构设计，包含四个层级：

UART通信层 (ft_servo_uart)：底层串口通信接口
硬件抽象层 (ft_servo_hw)：硬件交互接口，管理发送和接收缓冲区
协议层 (ft_servo_protocol)：实现飞特舵机通信协议
应用层 (ft_servo_app)：提供高级API，便于舵机控制
├── ft_servo_uart.h/c     - UART通信层
├── ft_servo_hw.h/c       - 硬件抽象层
├── ft_servo_protocol.h/c - 协议层
├── ft_servo_app.h/c      - 应用层
└── ft_servo_example.c    - 示例应用程序
功能特性

支持飞特SMS系列舵机
支持位置控制和恒速模式
支持同步控制多个舵机
支持异步写位置控制
支持读取舵机状态信息（位置、速度、负载、电压、温度、电流等）
支持舵机扫描识别
支持批量读取舵机信息
使用方法
初始化
```
// 假设使用UART3连接舵机
extern UART_HandleTypeDef huart3;

// 初始化舵机库
ft_servo_app_init(&huart3);
扫描舵机
cCopy// 扫描舵机
int servo_id = ft_protocol_ping(1);  // 检查ID为1的舵机是否存在
if (servo_id >= 0) {
    // 舵机存在
}
位置控制
cCopy// 参数：ID, 位置, 速度限制, 加速度
ft_servo_app_write_pos(1, 500, 100, 10);  // ID为1的舵机移动到位置500，速度100，加速度10
恒速模式
cCopy// 设置为恒速模式
ft_servo_app_wheel_mode(1);

// 控制转速：ID, 速度, 加速度
ft_servo_app_write_speed(1, 500, 10);  // 正转，速度500，加速度10
ft_servo_app_write_speed(1, -500, 10); // 反转，速度500，加速度10
ft_servo_app_write_speed(1, 0, 10);    // 停止
读取舵机信息
cCopy// 单独读取信息
int position = ft_servo_app_read_pos(1);
int speed = ft_servo_app_read_speed(1);
int load = ft_servo_app_read_load(1);
int voltage = ft_servo_app_read_voltage(1);
int temperature = ft_servo_app_read_temperature(1);
int moving = ft_servo_app_read_move(1);
int current = ft_servo_app_read_current(1);

// 批量读取所有信息
if (ft_servo_app_feedback(1) > 0) {
    position = ft_servo_app_read_pos(0xFF);  // 使用0xFF表示使用缓存数据
    speed = ft_servo_app_read_speed(0xFF);
    // ...其他信息读取
}
同步控制多个舵机
cCopyuint8_t ids[3] = {1, 2, 3};
int16_t positions[3] = {100, 200, 300};
uint16_t speeds[3] = {100, 100, 100};
uint8_t accs[3] = {10, 10, 10};

// 同时控制3个舵机移动到不同位置
ft_servo_app_sync_write_pos(ids, 3, positions, speeds, accs);
异步写位置控制
cCopy// 设置异步写指令
ft_servo_app_reg_write_pos(1, 100, 100, 10);
ft_servo_app_reg_write_pos(2, 200, 100, 10);
ft_servo_app_reg_write_pos(3, 300, 100, 10);

// 执行所有异步写指令
ft_servo_app_reg_write_action();
```
## 数据帧格式说明
SMS舵机通信协议帧格式如下：

标准指令帧格式：
Copy0xFF 0xFF ID LEN INST PARAM1...PARAMN CHECK

0xFF 0xFF: 帧头固定为两个0xFF
ID: 舵机ID (0~253)，0xFE为广播ID
LEN: 数据长度 (参数字节数+2)
INST: 指令类型

0x01: PING
0x02: READ
0x03: WRITE
0x04: REG_WRITE
0x05: REG_ACTION
0x83: SYNC_WRITE


PARAM1...PARAMN: 参数
CHECK: 校验和，为除0xFF外所有字节的累加和取反


写位置指令帧示例 (ID=1, 位置=500, 速度=200, 加速度=10)：
Copy0xFF 0xFF 0x01 0x09 0x03 0x29 0x0A 0xF4 0x01 0x00 0x00 0xC8 0x00 0xFE
解释：

0xFF 0xFF: 帧头
0x01: ID=1
0x09: 长度=9 (7个参数+2)
0x03: 写指令
0x29: 起始地址(ACC寄存器)
0x0A: 加速度=10
0xF4 0x01: 位置=500 (小端格式)
0x00 0x00: 时间=0 (SMS舵机不使用)
0xC8 0x00: 速度=200 (小端格式)
0xFE: 校验和



## 注意事项

使用前需确保舵机波特率与串口配置匹配，默认为115200bps
舵机ID默认为1，使用前请先确认舵机ID设置
控制速度和加速度时需注意舵机规格限制，避免超出范围
SMS舵机位置模式下，位置范围通常为01000，恒速模式下，速度范围通常为-10001000





## 适配说明
本库已针对DJI C型开发板（STM32F4系列）进行了优化，如需用于其他平台，主要修改以下文件：

ft_servo_uart.c：修改串口初始化和通信函数
ft_servo_hw.c：如有必要，调整缓冲区大小


