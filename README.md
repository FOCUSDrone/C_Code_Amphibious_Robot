# 注意事项：
- car相关一些代码已经可以运行并且有真机测试，but不够robust，Feiziben还在优化他。
- transform相关代码还需要真机测试，但是我们认为他可行，于是先行加入
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
- cubemx配置等
## [FREETOS快速上手教程](https://github.com/FOCUSDrone/C_code_amphibious_robot/blob/main/FreeRTOS%E5%AE%8C%E5%85%A8%E5%BC%80%E5%8F%91%E6%89%8B%E5%86%8C%E4%B9%8B%E4%B8%8A%E5%86%8C_%E5%BF%AB%E9%80%9F%E5%85%A5%E9%97%A8.pdf)
- FreeTos实时操作系统的资料
## [C板用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)


## 代码架构讲解
```
├── Application/                # 应用层，所有的线程（包括中断）都在这里
 │   ├── Inc/                    # 存放头文件
 │   ├── Src/                    # 存放源文件
 │   │   ├── buzzer_task.c       # 蜂鸣器线程，根据自检线程反映数据，实时报警
 │   │   ├── CAN_receive.c       # CAN总线接收线程
 │   │   ├── detect_task.c       # 自检线程线程，检测各个模块运行状态
 │   │   ├── INS_task.c          # 惯性导航系统线程，运行姿态融合解算算法
 │   │   ├── key_task.c          # 按键线程，debug用途
 │   │   ├── led_task.c          # LED灯线程，三色光流动效果，可以反应系统是否死机
 │   │   ├── remote_receive.c    # 遥控器接收线程，串口usart3中断
 │   │   ├── servo_receive.c     # 舵机接收线程，串口usart1中断
 │   │   ├── test_task.c         # 测试线程，dubug用途
 │   │   └── usb_task.c          # USB虚拟串口与上位机通信线程
 ├── BSP/                        # DJI的C板底层库，是对Core底层的用户封装，构建应用层，导入库需要用此处的函数重构
 ├── Core/                       # CubeMX生成文件，包括main和硬件底层，这里的文件都不需要修改
 ├── Devices/                    # 一些复杂设备的驱动，主要是陀螺仪和压力计芯片的驱动（操作其他芯片都放这里）
 ├── Drivers/                    # CubeMX生成文件，包括freertos操作系统驱动，CMSIS驱动，不需要修改
 ├── Library/                    # 第三方库和自定义算法（包括舵机、数学库）
 ├── MDK-ARM/                    # CubeMX生成文件, IDE配置文件夹，用于keil的编译和储存编译后用于烧录的文件不需要修改
 ├── Middlewares/                # CubeMX生成文件, 中间软件，包括freetos、数学库（CMSIS，相当底层芯片的运算加速）
 └── USB_DEVICE/                 # CubeMX生成文件, 和虚拟串口有关的库，不需要修改
```


