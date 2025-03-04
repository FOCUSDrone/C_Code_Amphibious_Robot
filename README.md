**工程文件结构**

```
2025EmbeddedDrone/
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
├── BSP/                        # DJI的C板底层库，是对Core底层的用户封装
├── Core/                       # CubeMX生成文件，包括main和硬件底层，这里的文件都不需要修改
├── Devices/                    # 一些复杂设备的驱动，主要是陀螺仪和压力计芯片的驱动
├── Drivers/                    # CubeMX生成文件，包括freertos操作系统驱动，CMSIS驱动，不需要修改
├── Library/                    # 第三方库和自定义算法
├── MDK-ARM/                    # CubeMX生成文件, IDE配置文件夹，不需要修改
├── Middlewares/                # CubeMX生成文件, 中间软件
└── USB_DEVICE/                 # CubeMX生成文件, 和虚拟串口有关的库，不需要修改
```



