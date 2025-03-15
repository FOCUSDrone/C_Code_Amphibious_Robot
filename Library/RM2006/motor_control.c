/**
  ****************************(C) COPYRIGHT 2025 DJI****************************
  * @file       motor_control.c
  * @brief      电机控制层，处理电机速度控制及PID调节
  * @note       作为car_task的底层模块
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-14      Feiziben          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 DJI****************************
  */

#include "motor_control.h"
#include "can_receive.h"
#include <math.h>


/* PID控制结构体 */
typedef struct {
    fp32 kp;                  // 比例系数
    fp32 ki;                  // 积分系数
    fp32 kd;                  // 微分系数
    
    fp32 error;               // 当前误差
    fp32 last_error;          // 上次误差
    fp32 integral;            // 积分项
    
    fp32 max_out;             // 最大输出限制
    fp32 max_integral;        // 最大积分限制
    
    fp32 out;                 // 输出值
} motor_pid_t;

/* 电机控制结构体 */
typedef struct {
    /* 电机目标数据 */
    fp32 left_speed_set;     // 左电机目标速度
    fp32 right_speed_set;    // 右电机目标速度
    
    /* 电机控制数据 */
    int16_t left_current;    // 左电机控制电流
    int16_t right_current;   // 右电机控制电流
    
    /* 电机测量数据指针 */
    const motor_measure_t *left_motor;  // 左电机数据
    const motor_measure_t *right_motor; // 右电机数据
} motor_control_t;



/* 全局变量 */
static motor_control_t motor_control;    // 电机控制数据
static motor_pid_t motor_pid[2];         // 左右电机PID控制器

/* 函数声明 */
static void pid_init(motor_pid_t *pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_integral);
static fp32 pid_calc(motor_pid_t *pid, fp32 measure, fp32 target);

/**
  * @brief          PID控制器初始化
  * @param[in]      pid: PID控制器指针
  * @param[in]      kp: 比例系数
  * @param[in]      ki: 积分系数
  * @param[in]      kd: 微分系数
  * @param[in]      max_out: 最大输出
  * @param[in]      max_integral: 积分限幅
  * @retval         none
  */
static void pid_init(motor_pid_t *pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_integral)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_integral = max_integral;
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}

/**
  * @brief          PID计算
  * @param[in]      pid: PID控制器指针
  * @param[in]      measure: 测量值
  * @param[in]      target: 目标值
  * @retval         控制输出
  */
static fp32 pid_calc(motor_pid_t *pid, fp32 measure, fp32 target)
{
    pid->error = target - measure;
    
    // 积分项更新
    pid->integral += pid->error;
    
    // 积分限幅
    if (pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }
    else if (pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }
    
    // PID计算
    pid->out = pid->kp * pid->error + 
               pid->ki * pid->integral + 
               pid->kd * (pid->error - pid->last_error);
    
    // 输出限幅
    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    else if (pid->out < -pid->max_out)
    {
        pid->out = -pid->max_out;
    }
    
    // 保存上次误差
    pid->last_error = pid->error;
    
    return pid->out;
}

/**
  * @brief          电机控制初始化
  * @param[in]      none
  * @retval         none
  */
void motor_control_init(void)
{
    // 初始化左电机PID
    pid_init(&motor_pid[0], MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, 
             MOTOR_CURRENT_MAX, MOTOR_CURRENT_MAX / 2);
    
    // 初始化右电机PID
    pid_init(&motor_pid[1], MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, 
             MOTOR_CURRENT_MAX, MOTOR_CURRENT_MAX / 2);
    
    // 获取电机数据指针
    motor_control.left_motor = get_chassis_motor1_measure_point();  // 获取左电机数据
    motor_control.right_motor = get_chassis_motor2_measure_point(); // 获取右电机数据
	
    
    // 初始化控制参数
    motor_control.left_speed_set = 0.0f;
    motor_control.right_speed_set = 0.0f;
    motor_control.left_current = 0;
    motor_control.right_current = 0;
}

/**
  * @brief          设置左右电机速度
  * @param[in]      left_speed: 左电机目标速度，范围[-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]
  * @param[in]      right_speed: 右电机目标速度，范围[-MOTOR_SPEED_MAX, MOTOR_SPEED_MAX]
  * @retval         none
  */
void motor_speed_set(fp32 left_speed, fp32 right_speed)
{
    // 限制速度范围
    if (left_speed > MOTOR_SPEED_MAX)
    {
        left_speed = MOTOR_SPEED_MAX;
    }
    else if (left_speed < -MOTOR_SPEED_MAX)
    {
        left_speed = -MOTOR_SPEED_MAX;
    }
    
    if (right_speed > MOTOR_SPEED_MAX)
    {
        right_speed = MOTOR_SPEED_MAX;
    }
    else if (right_speed < -MOTOR_SPEED_MAX)
    {
        right_speed = -MOTOR_SPEED_MAX;
    }
    
    // 设置目标速度，考虑传动比
    motor_control.left_speed_set = left_speed * MOTOR_GEAR_RATIO;
    // 右电机方向取反（由于安装方向）
    motor_control.right_speed_set = -right_speed * MOTOR_GEAR_RATIO;
}

/**
  * @brief          电机控制计算与执行，在周期任务中调用
  * @param[in]      none
  * @retval         none
  */
void motor_control_calc(void)
{
    // 使用PID计算左电机电流
    motor_control.left_current = (int16_t)pid_calc(&motor_pid[0], 
                                                motor_control.left_motor->speed_rpm, 
                                                motor_control.left_speed_set);
    
    // 使用PID计算右电机电流
    motor_control.right_current = (int16_t)pid_calc(&motor_pid[1], 
                                                 motor_control.right_motor->speed_rpm, 
                                                 motor_control.right_speed_set);
    
    // 发送控制命令到两个电机
    CAN_cmd_chassis(motor_control.left_current, motor_control.right_current);
}