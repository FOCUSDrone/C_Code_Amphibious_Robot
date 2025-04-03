/**
  ******************************************************************************
  * @file       adsorption_task.c
  * @brief      吸附线程，通过遥控器控制风扇转速
  * @note       使用遥控器通道3和开关控制，避免与car_task冲突
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-25      Feiziben        1. 完成
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
  
#include "adsorption_task.h"
#include "cmsis_os.h"
#include "remote_receive.h"
#include "arm_math.h"
#include "user_lib.h"
#include "bsp_fan.h"

/* 内部变量 */
static fan_control_t fan_control;

/* 风扇控制相关参数 */
#define FAN_RPM_MIN               500     /* 最小有效转速 */
#define FAN_REMOTE_DEADBAND       100     /* 遥控器死区值 */
#define FAN_REMOTE_MAX_VALUE      660     /* 遥控器最大有效值 */
#define FAN_CONTROL_RAMPUP_STEP   500     /* 风扇启动加速斜率(RPM/控制周期) */
#define FAN_CONTROL_RAMPDN_STEP   300     /* 风扇减速斜率(RPM/控制周期) */

/**
  * @brief          遥控器值映射到风扇转速
  * @param[in]      remote_val: 遥控器通道值
  * @retval         风扇目标转速
  */
static uint16_t map_remote_to_rpm(int16_t remote_val)
{
    int16_t abs_remote_val;
    uint16_t target_rpm = 0;
    
    /* 获取绝对值，允许正反向控制 */
    abs_remote_val = remote_val > 0 ? remote_val : -remote_val;
    
    /* 死区处理 */
    if (abs_remote_val < FAN_REMOTE_DEADBAND)
    {
        return 0;
    }
    
    /* 限制遥控器值范围 */
    if (abs_remote_val > FAN_REMOTE_MAX_VALUE)
    {
        abs_remote_val = FAN_REMOTE_MAX_VALUE;
    }
    
    /* 线性映射：遥控器值到转速范围 */
    target_rpm = (uint16_t)((abs_remote_val - FAN_REMOTE_DEADBAND) * 
                           (FAN_RPM_MAX - FAN_RPM_MIN)/ 
                           (FAN_REMOTE_MAX_VALUE - FAN_REMOTE_DEADBAND) + 
                           FAN_RPM_MIN);
    
    return target_rpm;
}

/**
  * @brief          平滑控制风扇转速，防止突变
  * @param[in]      target_rpm: 目标转速
  * @param[in]      current_rpm: 当前转速
  * @retval         平滑后的目标转速
  */
static uint16_t smooth_rpm_control(uint16_t target_rpm, uint16_t current_rpm)
{
    uint16_t smooth_rpm;
    
    /* 增速限制 */
    if (target_rpm > current_rpm)
    {
        if (target_rpm - current_rpm > FAN_CONTROL_RAMPUP_STEP)
        {
            smooth_rpm = current_rpm + FAN_CONTROL_RAMPUP_STEP;
        }
        else
        {
            smooth_rpm = target_rpm;
        }
    }
    /* 减速限制 */
    else if (target_rpm < current_rpm)
    {
        if (current_rpm - target_rpm > FAN_CONTROL_RAMPDN_STEP)
        {
            smooth_rpm = current_rpm - FAN_CONTROL_RAMPDN_STEP;
        }
        else
        {
            smooth_rpm = target_rpm;
        }
    }
    /* 无需变化 */
    else
    {
        smooth_rpm = target_rpm;
    }
    
    return smooth_rpm;
}

/**
  * @brief          获取风扇控制状态
  * @param[in]      none
  * @retval         风扇控制状态结构体指针
  */
const fan_control_t *get_fan_control_point(void)
{
    return &fan_control;
}

/**
  * @brief          adsorption任务，在RTOS中注册为线程
  * @param[in]      argument: 线程参数
  * @retval         none
  */
void adsorption_task(void const * argument)
{
    /* 本地变量 */
    int16_t remote_val;
    uint16_t raw_target_rpm;
    uint16_t smooth_target_rpm;
    const int16_t *remote_ch;
    int16_t switch_position;
    fan_control_mode_e last_mode = FAN_MODE_OFF;
    
    /* 初始化风扇 */
    fan_init();
    
    /* 初始化控制状态 */
    fan_control.mode = FAN_MODE_OFF;
    fan_control.target_rpm = 0;
    fan_control.current_rpm = 0;
    fan_control.remote_value = 0;
    fan_control.is_active = 0;
    
    /* 延时等待系统稳定 */
    vTaskDelay(500);
    
    /* 主循环 */
    while (1)
    {
        /* 获取遥控器数据 */
        remote_ch = get_remote_ch_point();
        
        /* 读取遥控器通道值 */
        remote_val = remote_ch[FAN_CONTROL_CHANNEL];
        fan_control.remote_value = remote_val;
        
        /* 读取控制开关位置 */
        switch_position = remote_ch[FAN_CONTROL_SWITCH + 4]; /* 开关在通道5和6 */
        
        /* 根据开关位置确定控制模式 */
        if (switch_is_down(switch_position))
        {
            fan_control.mode = FAN_MODE_OFF;
        }
        else if (switch_is_mid(switch_position))
        {
            fan_control.mode = FAN_MODE_MANUAL;
        }
        else if (switch_is_up(switch_position))
        {
            fan_control.mode = FAN_MODE_AUTO;
        }
        
        /* 检测模式切换 */
        if (fan_control.mode != last_mode)
        {
            /* 切换到关闭模式时停止风扇 */
            if (fan_control.mode == FAN_MODE_OFF)
            {
                fan_stop();
                fan_control.target_rpm = 0;
                fan_control.is_active = 0;
            }
            /* 从关闭模式切换到其他模式时重新初始化 */
            else if (last_mode == FAN_MODE_OFF)
            {
                fan_control.is_active = 1;
            }
            
            last_mode = fan_control.mode;
        }
        
        /* 根据控制模式处理 */
        if (fan_control.mode == FAN_MODE_MANUAL && fan_control.is_active)
        {
            /* 映射遥控器值到转速 */
            raw_target_rpm = map_remote_to_rpm(remote_val);
            
            /* 平滑控制转速变化 */
            smooth_target_rpm = smooth_rpm_control(raw_target_rpm, fan_control.target_rpm);
            
            /* 设置风扇转速 */
            fan_set_rpm(smooth_target_rpm);
            
            /* 更新目标转速 */
            fan_control.target_rpm = smooth_target_rpm;
        }
        else if (fan_control.mode == FAN_MODE_AUTO && fan_control.is_active)
        {
            /* 自动模式预留，此处可以基于传感器反馈实现自动控制 */
            /* 这里简单设置一个固定转速作为示例 */
            smooth_target_rpm = smooth_rpm_control(FAN_RPM_MAX / 2, fan_control.target_rpm);
            fan_set_rpm(smooth_target_rpm);
            fan_control.target_rpm = smooth_target_rpm;
        }
        
        /* 获取当前转速 */
        fan_control.current_rpm = fan_get_rpm();
        
        /* 调用风扇控制处理函数 */
        fan_process();
        
        /* 控制频率 */
        vTaskDelay(ADSORPTION_TASK_TIME);
    }
}
