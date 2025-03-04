/**
  ******************************************************************************
  * @file       Adhesion_task.c
  * @brief      吸附系统控制任务，基于风机驱动实现可控吸附功能
  * @note       使用FreeRTOS实现，依赖bsp_fan驱动
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. 调整中
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "Adhesion_task.h"
#include "remote_receive.h"
#include "detect_task.h"
#include "cmsis_os.h"

/* 任务句柄 */
osThreadId adhesion_task_handle;

/* 静态全局变量 */
static adhesion_info_t adhesion_info = {0};
static uint32_t last_control_time = 0;
static uint8_t is_initialized = 0;

/* 静态函数声明 */
static void adhesion_control_update(void);
static void adhesion_mode_switch(void);
static float adhesion_rc_to_power(int16_t rc_value);
static void adhesion_safety_check(void);

/**
  * @brief          吸附系统任务，通过遥控器控制风机实现吸附功能
  * @param[in]      argument: 任务参数指针
  * @retval         void
  */
void Adhesion_task(void const * argument)
{
    /* 等待其他模块初始化完成 */
    vTaskDelay(ADHESION_TASK_INIT_TIME);
    
    /* 初始化检查 */
    if (!is_initialized)
    {
        adhesion_info.state = ADHESION_STATE_ERROR;
        adhesion_info.error_code = 1; // 未初始化错误
        while (1)
        {
            vTaskDelay(1000);
        }
    }
    
    /* 获取遥控器数据指针 */
    adhesion_info.rc_data = get_remote_ch_point();
    
    /* 任务主循环 */
    uint32_t wake_time = osKernelSysTick();
    
    while (1)
    {
        /* 检查遥控器数据有效性 */
        if (toe_is_error(DBUS_TOE))
        {
            /* 遥控器断连，安全停止 */
            adhesion_emergency_stop();
        }
        else
        {
            /* 处理模式切换 */
            adhesion_mode_switch();
            
            /* 更新控制逻辑 */
            adhesion_control_update();
            
            /* 安全检查 */
            adhesion_safety_check();
        }
        
        /* 更新系统运行时间 */
        if (adhesion_info.state == ADHESION_STATE_RUNNING)
        {
            adhesion_info.runtime += ADHESION_TASK_PERIOD;
        }
        
        /* 任务循环延时 */
        wake_time += ADHESION_TASK_PERIOD;
        osDelayUntil(&wake_time);
    }
}

/**
  * @brief          吸附系统初始化
  * @param[in]      fan_tim: 风机PWM定时器句柄
  * @param[in]      fan_channel: 风机PWM通道
  * @retval         0成功，非0失败
  */
uint8_t adhesion_system_init(TIM_HandleTypeDef* fan_tim, uint32_t fan_channel)
{
    /* 初始化风机驱动 */
    if (bsp_fan_init(fan_tim, fan_channel) != 0)
    {
        return 1;
    }
    
    /* 初始化吸附系统状态 */
    adhesion_info.mode = ADHESION_MODE_OFF;
    adhesion_info.state = ADHESION_STATE_OFF;
    adhesion_info.power = 0.0f;
    adhesion_info.target_power = 0.0f;
    adhesion_info.runtime = 0;
    adhesion_info.fan_index = FAN_MAIN_INDEX;
    adhesion_info.error_code = 0;
    
    /* 创建任务 */
    osThreadDef(adhesionTask, Adhesion_task, ADHESION_TASK_PRIO, 0, ADHESION_STK_SIZE);
    adhesion_task_handle = osThreadCreate(osThread(adhesionTask), NULL);
    
    if (adhesion_task_handle == NULL)
    {
        return 2;
    }
    
    /* 标记初始化完成 */
    is_initialized = 1;
    
    return 0;
}

/**
  * @brief          设置吸附系统模式
  * @param[in]      mode: 吸附模式
  * @retval         0成功，非0失败
  */
uint8_t adhesion_set_mode(adhesion_mode_e mode)
{
    if (!is_initialized)
    {
        return 1;
    }
    
    /* 模式切换逻辑 */
    if (mode != adhesion_info.mode)
    {
        switch (mode)
        {
            case ADHESION_MODE_OFF:
                /* 关闭模式 - 停止风机 */
                bsp_fan_set_speed(adhesion_info.fan_index, 0);
                adhesion_info.state = ADHESION_STATE_STOPPING;
                adhesion_info.power = 0.0f;
                adhesion_info.target_power = 0.0f;
                break;
                
            case ADHESION_MODE_MANUAL:
                /* 手动模式 - 初始功率为0 */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 0.0f;
                break;
                
            case ADHESION_MODE_AUTO:
                /* 自动模式 - 默认50%功率 */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 50.0f;
                break;
                
            case ADHESION_MODE_SMART:
                /* 智能模式 - 自适应控制 */
                adhesion_info.state = ADHESION_STATE_STARTING;
                adhesion_info.target_power = 30.0f;
                break;
                
            default:
                return 2;
        }
        
        adhesion_info.mode = mode;
    }
    
    return 0;
}

/**
  * @brief          设置吸附功率
  * @param[in]      power: 功率百分比(0-100)
  * @retval         0成功，非0失败
  */
uint8_t adhesion_set_power(float power)
{
    if (!is_initialized)
    {
        return 1;
    }
    
    /* 限制功率范围 */
    if (power > 100.0f)
    {
        power = 100.0f;
    }
    else if (power < 0.0f)
    {
        power = 0.0f;
    }
    
    /* 更新目标功率 */
    adhesion_info.target_power = power;
    
    return 0;
}

/**
  * @brief          获取吸附系统信息
  * @param[out]     info: 吸附系统信息结构指针
  * @retval         0成功，非0失败
  */
uint8_t adhesion_get_info(adhesion_info_t* info)
{
    if (!is_initialized || info == NULL)
    {
        return 1;
    }
    
    /* 复制信息 */
    *info = adhesion_info;
    
    return 0;
}

/**
  * @brief          紧急停止吸附系统
  * @param[in]      none
  * @retval         none
  */
void adhesion_emergency_stop(void)
{
    /* 直接关闭风机 */
    bsp_fan_set_speed(adhesion_info.fan_index, 0);
    
    /* 更新状态 */
    adhesion_info.mode = ADHESION_MODE_OFF;
    adhesion_info.state = ADHESION_STATE_OFF;
    adhesion_info.power = 0.0f;
    adhesion_info.target_power = 0.0f;
}

/**
  * @brief          吸附控制更新函数
  * @param[in]      none
  * @retval         none
  */
static void adhesion_control_update(void)
{
    uint32_t current_time = osKernelSysTick();
    
    /* 控制更新间隔限制 */
    if (current_time - last_control_time < ADHESION_CONTROL_INTERVAL)
    {
        return;
    }
    last_control_time = current_time;
    
    /* 基于当前模式执行控制逻辑 */
    switch (adhesion_info.mode)
    {
        case ADHESION_MODE_OFF:
            /* 确保风机停止 */
            if (adhesion_info.power > 0.0f)
            {
                bsp_fan_set_speed(adhesion_info.fan_index, 0);
                adhesion_info.power = 0.0f;
            }
            
            if (adhesion_info.state == ADHESION_STATE_STOPPING)
            {
                adhesion_info.state = ADHESION_STATE_OFF;
            }
            break;
            
        case ADHESION_MODE_MANUAL:
            /* 手动控制模式 - 解析遥控器输入 */
            if (adhesion_info.rc_data != NULL)
            {
                float rc_power = adhesion_rc_to_power(adhesion_info.rc_data[ADHESION_POWER_CHANNEL]);
                adhesion_info.target_power = rc_power;
            }
            
            /* 更新风机速度 */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* 获取当前实际功率 */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* 更新状态 */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
            
        case ADHESION_MODE_AUTO:
            /* 自动控制模式 - 使用预设功率 */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* 获取当前实际功率 */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* 更新状态 */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
            
        case ADHESION_MODE_SMART:
            /* 
             * 智能控制模式 - 可以根据传感器数据自适应调整功率
             * 此处简化为自动控制，实际应用中可以添加传感器反馈逻辑
             */
            bsp_fan_set_target_speed(adhesion_info.fan_index, adhesion_info.target_power);
            bsp_fan_speed_ctrl_update();
            
            /* 获取当前实际功率 */
            adhesion_info.power = bsp_fan_get_speed(adhesion_info.fan_index);
            
            /* 更新状态 */
            if (adhesion_info.state == ADHESION_STATE_STARTING && 
                adhesion_info.power > 0.0f)
            {
                adhesion_info.state = ADHESION_STATE_RUNNING;
            }
            break;
    }
    
    /* 反馈设备状态 */
    detect_hook(ADHESION_TOE);
}

/**
  * @brief          吸附模式切换函数
  * @param[in]      none
  * @retval         none
  */
static void adhesion_mode_switch(void)
{
    /* 检查遥控器数据有效性 */
    if (adhesion_info.rc_data == NULL)
    {
        return;
    }
    
    /* 基于遥控器开关位置切换模式 */
    int16_t mode_channel = adhesion_info.rc_data[ADHESION_MODE_CHANNEL];
    
    if (switch_is_down(mode_channel))
    {
        /* 开关在下位置 - 关闭吸附 */
        adhesion_set_mode(ADHESION_MODE_OFF);
    }
    else if (switch_is_mid(mode_channel))
    {
        /* 开关在中间位置 - 自动模式 */
        adhesion_set_mode(ADHESION_MODE_AUTO);
    }
    else if (switch_is_up(mode_channel))
    {
        /* 开关在上位置 - 手动模式 */
        adhesion_set_mode(ADHESION_MODE_MANUAL);
    }
}

/**
  * @brief          将遥控器值转换为功率百分比
  * @param[in]      rc_value: 遥控器通道值(-660至660)
  * @retval         功率百分比(0-100)
  */
static float adhesion_rc_to_power(int16_t rc_value)
{
    /* 通常遥控器下拉为负值，上推为正值 */
    if (rc_value > -5 && rc_value < 5)
    {
        /* 遥控器在中间位置，应用死区 */
        return 0.0f;
    }
    else if (rc_value <= 0)
    {
        /* 遥控器下拉，映射为0-100%功率 */
        return (float)(-rc_value) / 660.0f * 100.0f;
    }
    else
    {
        /* 遥控器上推，定义为0功率 */
        return 0.0f;
    }
}

/**
  * @brief          吸附系统安全检查
  * @param[in]      none
  * @retval         none
  */
static void adhesion_safety_check(void)
{
    /* 
     * 此函数可以添加各种安全检查，例如:
     * 1. 电机温度监控
     * 2. 电流监控
     * 3. 卡死检测
     * 4. 电压监控
     * 
     * 此处为简化版本，实际应用中应扩展此功能
     */
     
    /* 示例: 运行时间过长时降低功率 */
    if (adhesion_info.runtime > 3600000) // 1小时
    {
        if (adhesion_info.target_power > 80.0f)
        {
            adhesion_info.target_power = 80.0f;
        }
    }
}