
  /**
  ******************************************************************************
  * @file       car_task.c
  * @brief      小车控制任务，基于遥控器实现差速小车控制
  * @note       使用FreeRTOS实现，依赖bsp_car驱动
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. 调试中
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "car_task.h"
#include "remote_receive.h"
#include "detect_task.h"
#include "cmsis_os.h"
#include <math.h>

/* 任务句柄 */
osThreadId car_task_handle;

/* 全局变量 */
static car_info_t car_info = {0};
static const int16_t* rc_data = NULL;
static uint8_t is_initialized = 0;
static uint32_t last_rc_time = 0;
static uint32_t last_timeout_check = 0;

/* 静态函数声明 */
static void car_mode_switch(void);
static void car_rc_control_update(void);
static void car_status_update(void);
static void car_safety_check(void);
static int16_t apply_deadband(int16_t value, int16_t deadband);
static float map_remote_to_speed(int16_t rc_value, float scale);

/**
  * @brief          小车控制任务，处理遥控器输入并控制小车运动
  * @param[in]      argument: 任务参数
  * @retval         void
  */
void car_task(void const * argument)
{
    /* 等待其他模块初始化完成 */
    vTaskDelay(CAR_TASK_INIT_TIME);

    /* 初始化检查 */
    if (!is_initialized)
    {
        car_info.state = CAR_STATE_ERROR;
        car_info.error_code = 1; // 初始化失败错误
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    /* 获取遥控器数据指针 */
    rc_data = get_remote_ch_point();
    if (rc_data == NULL)
    {
        car_info.state = CAR_STATE_ERROR;
        car_info.error_code = 2; // 遥控器指针获取失败
        while (1)
        {
            vTaskDelay(1000);
        }
    }

    /* 任务主循环 */
    uint32_t wake_time = osKernelSysTick();

    while (1)
    {
        /* 安全检查 */
        car_safety_check();

        if (car_info.rc_connected)
        {
            /* 处理模式切换 */
            car_mode_switch();

            /* 处理遥控器控制更新 */
            car_rc_control_update();
        }

        /* 更新小车状态 */
        car_status_update();

        /* 更新运行时间 */
        car_info.runtime += CAR_TASK_PERIOD;

        /* 任务周期延时 */
        wake_time += CAR_TASK_PERIOD;
        osDelayUntil(&wake_time);
    }
}

/**
  * @brief          小车控制系统初始化
  * @param[in]      left_tim: 左电机定时器句柄
  * @param[in]      left_channel: 左电机PWM通道
  * @param[in]      right_tim: 右电机定时器句柄
  * @param[in]      right_channel: 右电机PWM通道
  * @retval         0成功，非0失败
  */
uint8_t car_system_init(TIM_HandleTypeDef* left_tim, uint32_t left_channel,
                        TIM_HandleTypeDef* right_tim, uint32_t right_channel)
{
    /* 初始化底层驱动 */
    if (bsp_car_init(left_tim, left_channel, right_tim, right_channel) != 0)
    {
        return 1;
    }

    /* 初始化小车状态 */
    car_info.mode = CAR_MODE_STOP;
    car_info.state = CAR_STATE_STOP;
    car_info.forward_speed = 0.0f;
    car_info.turn_rate = 0.0f;
    car_info.left_speed = 0.0f;
    car_info.right_speed = 0.0f;
    car_info.runtime = 0;
    car_info.error_code = 0;
    car_info.rc_connected = 0;

    /* 设置电机方向 */
    bsp_car_set_motor_direction(CAR_LEFT_MOTOR_INDEX, MOTOR_DIR_NORMAL);
    bsp_car_set_motor_direction(CAR_RIGHT_MOTOR_INDEX, MOTOR_DIR_NORMAL);

    /* 创建任务 */
    osThreadDef(carTask, car_task, CAR_TASK_PRIO, 0, CAR_STK_SIZE);
    car_task_handle = osThreadCreate(osThread(carTask), NULL);

    if (car_task_handle == NULL)
    {
        return 2;
    }

    /* 标记初始化完成 */
    is_initialized = 1;

    return 0;
}

/**
  * @brief          设置小车控制模式
  * @param[in]      mode: 控制模式
  * @retval         0成功，非0失败
  */
uint8_t car_set_mode(car_mode_e mode)
{
    if (!is_initialized)
    {
        return 1;
    }

    /* 模式切换逻辑 */
    if (mode != car_info.mode)
    {
        switch (mode)
        {
            case CAR_MODE_STOP:
                /* 停止小车 */
                bsp_car_stop();
                car_info.forward_speed = 0.0f;
                car_info.turn_rate = 0.0f;
                break;

            case CAR_MODE_NORMAL:
                /* 普通模式 - 标准响应速度 */
                break;

            case CAR_MODE_SPORT:
                /* 运动模式 - 高灵敏度 */
                break;

            case CAR_MODE_ECO:
                /* 经济模式 - 低速省电 */
                break;

            case CAR_MODE_AUTO:
                /* 自动模式 - 自主控制 */
                break;

            case CAR_MODE_EMERGENCY:
                /* 紧急模式 - 立即停车 */
                car_emergency_stop();
                return 0;

            default:
                return 2;
        }

        car_info.mode = mode;
    }

    return 0;
}

/**
  * @brief          手动控制小车运动
  * @param[in]      forward_speed: 前进速度(-100至100)
  * @param[in]      turn_rate: 转向速率(-100至100)
  * @retval         0成功，非0失败
  */
uint8_t car_manual_control(float forward_speed, float turn_rate)
{
    if (!is_initialized)
    {
        return 1;
    }

    /* 确保在手动或普通模式下才接受控制 */
    if (car_info.mode == CAR_MODE_AUTO || car_info.mode == CAR_MODE_EMERGENCY)
    {
        return 2;
    }

    /* 设置小车运动 */
    if (bsp_car_set_motion(forward_speed, turn_rate) != 0)
    {
        return 3;
    }

    /* 更新状态信息 */
    car_info.forward_speed = forward_speed;
    car_info.turn_rate = turn_rate;

    return 0;
}

/**
  * @brief          获取小车控制信息
  * @param[out]     info: 信息结构体指针
  * @retval         0成功，非0失败
  */
uint8_t car_get_info(car_info_t* info)
{
    if (!is_initialized || info == NULL)
    {
        return 1;
    }

    /* 复制信息 */
    *info = car_info;

    return 0;
}

/**
  * @brief          紧急停止小车
  * @param[in]      none
  * @retval         none
  */
void car_emergency_stop(void)
{
    /* 立即停止所有电机 */
    bsp_car_stop();

    /* 更新状态 */
    car_info.mode = CAR_MODE_EMERGENCY;
    car_info.state = CAR_STATE_STOP;
    car_info.forward_speed = 0.0f;
    car_info.turn_rate = 0.0f;
    car_info.left_speed = 0.0f;
    car_info.right_speed = 0.0f;
}

/**
  * @brief          处理遥控器模式切换
  * @param[in]      none
  * @retval         none
  */
static void car_mode_switch(void)
{
    /* 检查遥控器数据有效性 */
    if (rc_data == NULL)
    {
        return;
    }

    /* 紧急停止通道检查 */
    if (switch_is_up(rc_data[CAR_EMERGENCY_CHANNEL]))
    {
        car_emergency_stop();
        return;
    }

    /* 基于模式切换通道设置模式 */
    int16_t mode_channel = rc_data[CAR_MODE_SWITCH_CHANNEL];

    if (switch_is_down(mode_channel))
    {
        /* 下位置 - 停止模式 */
        car_set_mode(CAR_MODE_STOP);
    }
    else if (switch_is_mid(mode_channel))
    {
        /* 中间位置 - 正常模式 */
        car_set_mode(CAR_MODE_NORMAL);
    }
    else if (switch_is_up(mode_channel))
    {
        /* 上位置 - 根据其他通道决定模式 */
        if (rc_data[CAR_FORWARD_CHANNEL] > 300)
        {
            /* 高速模式 */
            car_set_mode(CAR_MODE_SPORT);
        }
        else if (rc_data[CAR_FORWARD_CHANNEL] < -300)
        {
            /* 经济模式 */
            car_set_mode(CAR_MODE_ECO);
        }
        else
        {
            /* 默认为正常模式 */
            car_set_mode(CAR_MODE_NORMAL);
        }
    }
}

/**
  * @brief          处理遥控器控制更新
  * @param[in]      none
  * @retval         none
  */
static void car_rc_control_update(void)
{
    if (rc_data == NULL ||
        car_info.mode == CAR_MODE_STOP ||
        car_info.mode == CAR_MODE_EMERGENCY ||
        car_info.mode == CAR_MODE_AUTO)
    {
        return;
    }