/**
  ******************************************************************************
  * @file       Adhesion_task.h
  * @brief      吸附系统控制任务，基于风机驱动实现可控吸附功能
  * @note       使用FreeRTOS实现，依赖bsp_fan驱动
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   YourName    1. 完成
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#ifndef ADHESION_TASK_H
#define ADHESION_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_fan.h"

/* 任务参数配置 */
#define ADHESION_TASK_PRIO             15       // 任务优先级
#define ADHESION_STK_SIZE              128      // 任务栈大小
#define ADHESION_TASK_INIT_TIME        100      // 初始化延时(ms)
#define ADHESION_TASK_PERIOD           10       // 任务周期(ms)

/* 控制参数配置 */
#define ADHESION_MODE_CHANNEL          5        // 吸附模式切换通道
#define ADHESION_POWER_CHANNEL         2        // 吸附功率控制通道
#define ADHESION_SAFE_TEMP             70       // 安全温度阈值(°C)
#define ADHESION_CONTROL_INTERVAL      50       // 控制更新间隔(ms)

/* 吸附模式定义 */
typedef enum {
    ADHESION_MODE_OFF = 0,             // 关闭模式
    ADHESION_MODE_MANUAL,              // 手动控制模式
    ADHESION_MODE_AUTO,                // 自动控制模式
    ADHESION_MODE_SMART                // 智能控制模式
} adhesion_mode_e;

/* 吸附状态定义 */
typedef enum {
    ADHESION_STATE_OFF = 0,            // 关闭状态
    ADHESION_STATE_STARTING,           // 启动中
    ADHESION_STATE_RUNNING,            // 运行中
    ADHESION_STATE_STOPPING,           // 停止中
    ADHESION_STATE_ERROR               // 错误状态
} adhesion_state_e;

/* 吸附控制数据结构 */
typedef struct {
    adhesion_mode_e     mode;          // 当前控制模式
    adhesion_state_e    state;         // 当前状态
    float               power;         // 当前功率(%)
    float               target_power;  // 目标功率(%)
    uint32_t            runtime;       // 运行时间(ms)
    uint8_t             fan_index;     // 风机索引
    uint8_t             error_code;    // 错误代码
    const int16_t*      rc_data;       // 遥控器数据指针
} adhesion_info_t;

/* 函数声明 */
/**
  * @brief          吸附系统任务，通过遥控器控制风机实现吸附功能
  * @param[in]      argument: 任务参数指针
  * @retval         void
  */
extern void Adhesion_task(void const * argument);

/**
  * @brief          吸附系统初始化
  * @param[in]      fan_tim: 风机PWM定时器句柄
  * @param[in]      fan_channel: 风机PWM通道
  * @retval         0成功，非0失败
  */
extern uint8_t adhesion_system_init(TIM_HandleTypeDef* fan_tim, uint32_t fan_channel);

/**
  * @brief          设置吸附系统模式
  * @param[in]      mode: 吸附模式
  * @retval         0成功，非0失败
  */
extern uint8_t adhesion_set_mode(adhesion_mode_e mode);

/**
  * @brief          设置吸附功率
  * @param[in]      power: 功率百分比(0-100)
  * @retval         0成功，非0失败
  */
extern uint8_t adhesion_set_power(float power);

/**
  * @brief          获取吸附系统信息
  * @param[out]     info: 吸附系统信息结构指针
  * @retval         0成功，非0失败
  */
extern uint8_t adhesion_get_info(adhesion_info_t* info);

/**
  * @brief          紧急停止吸附系统
  * @param[in]      none
  * @retval         none
  */
extern void adhesion_emergency_stop(void);

#endif /* ADHESION_TASK_H */

