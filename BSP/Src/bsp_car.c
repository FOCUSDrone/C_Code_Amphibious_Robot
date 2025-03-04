/**
  ******************************************************************************
  * @file       bsp_car.c
  * @brief      С���������BSP�㣬֧�ֲ���ת��
  * @note       ����STM32 HAL�⣬ʹ��PWM���Ƶ��
  * @history
  *  Version    Date        Author      Modification
  *  V1.0.0     2025.3.04   Feiziben    1. ������
  *
  @verbatim
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "bsp_car.h"
#include <math.h>

/* ��̬����������� */
static motor_pwm_t motor_pwm[CAR_MOTOR_NUM];

/* С����ǰ״̬ */
static float motor_current_speed[CAR_MOTOR_NUM] = {0.0f, 0.0f}; // ��ǰ����ٶ�
static float motor_target_speed[CAR_MOTOR_NUM] = {0.0f, 0.0f};  // Ŀ�����ٶ�
static car_ctrl_mode_e car_ctrl_mode = CAR_CTRL_MANUAL;          // ����ģʽ
static float car_forward_speed = 0.0f;    // С��ǰ���ٶ�
static float car_turn_rate = 0.0f;        // С��ת������
static float car_target_forward = 0.0f;   // Ŀ��ǰ���ٶ�
static float car_target_turn = 0.0f;      // Ŀ��ת������

/**
  * @brief          �ٶ�ֵת��ΪPWMֵ
  * @param[in]      speed: �ٶ�ֵ(-100��100)
  * @param[in]      motor_dir: �������
  * @retval         PWMֵ(1000-2000)
  */
static uint32_t motor_speed_to_pwm(float speed, motor_dir_e motor_dir)
{
    /* �����ٶȷ�Χ */
    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < -100.0f)
    {
        speed = -100.0f;
    }
    
    /* Ӧ�õ������ת */
    if (motor_dir == MOTOR_DIR_REVERSE)
    {
        speed = -speed;
    }
    
    /* ����PWMֵ */
    uint32_t pwm;
    if (speed > 0)
    {
        pwm = CAR_PWM_NEUTRAL + (uint32_t)((speed / 100.0f) * (CAR_PWM_FORWARD_MAX - CAR_PWM_NEUTRAL));
    }
    else if (speed < 0)
    {
        pwm = CAR_PWM_NEUTRAL - (uint32_t)(((-speed) / 100.0f) * (CAR_PWM_NEUTRAL - CAR_PWM_BACKWARD_MAX));
    }
    else
    {
        pwm = CAR_PWM_NEUTRAL;
    }
    
    return pwm;
}

/**
  * @brief          PWMֵת��Ϊ΢��ֵ
  * @param[in]      htim: ��ʱ�����ָ��
  * @param[in]      us_value: ΢��ֵ
  * @retval         ��ʱ���Ƚ�ֵ
  */
static uint32_t pwm_us_to_compare(TIM_HandleTypeDef* htim, uint32_t us_value)
{
    uint32_t period = htim->Init.Period;
    uint32_t freq = CAR_PWM_TIM_FREQ;
    
    /* ����Ƚ�ֵ */
    return (uint32_t)((float)us_value * (float)freq / 1000000.0f);
}

/**
  * @brief          С��BSP��ʼ��
  * @param[in]      left_motor: ������ʱ�����
  * @param[in]      left_channel: ����PWMͨ��
  * @param[in]      right_motor: �ҵ����ʱ�����
  * @param[in]      right_channel: �ҵ��PWMͨ��
  * @retval         ����0��ʾ��ʼ���ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_init(TIM_HandleTypeDef* left_motor, uint32_t left_channel,
                   TIM_HandleTypeDef* right_motor, uint32_t right_channel)
{
    /* ������� */
    if (left_motor == NULL || right_motor == NULL)
    {
        return 1;
    }
    
    /* ����PWMʱ�� */
    uint32_t prescaler = HAL_RCC_GetSysClockFreq() / 2 / CAR_PWM_TIM_FREQ - 1;
    uint32_t period = CAR_PWM_PERIOD - 1;
    
    /* ����������ʱ�� */
    left_motor->Init.Prescaler = prescaler;
    left_motor->Init.Period = period;
    HAL_TIM_Base_Init(left_motor);
    
    /* �����ҵ����ʱ���������������ͬ�� */
    if (right_motor != left_motor)
    {
        right_motor->Init.Prescaler = prescaler;
        right_motor->Init.Period = period;
        HAL_TIM_Base_Init(right_motor);
    }
    
    /* ����PWMģʽ */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_us_to_compare(left_motor, CAR_PWM_NEUTRAL);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* ��������PWM */
    HAL_TIM_PWM_ConfigChannel(left_motor, &sConfigOC, left_channel);
    HAL_TIM_PWM_Start(left_motor, left_channel);
    
    /* �����ҵ��PWM */
    sConfigOC.Pulse = pwm_us_to_compare(right_motor, CAR_PWM_NEUTRAL);
    HAL_TIM_PWM_ConfigChannel(right_motor, &sConfigOC, right_channel);
    HAL_TIM_PWM_Start(right_motor, right_channel);
    
    /* �������� */
    motor_pwm[CAR_LEFT_MOTOR_INDEX].htim = left_motor;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].channel = left_channel;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].dir = MOTOR_DIR_NORMAL;
    motor_pwm[CAR_LEFT_MOTOR_INDEX].is_active = 1;
    
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].htim = right_motor;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].channel = right_channel;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].dir = MOTOR_DIR_NORMAL;
    motor_pwm[CAR_RIGHT_MOTOR_INDEX].is_active = 1;
    
    /* ���ó�ʼ״̬Ϊֹͣ */
    bsp_car_stop();
    
    return 0;
}

/**
  * @brief          ���õ������
  * @param[in]      motor_index: �������
  * @param[in]      dir: �������
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_set_motor_direction(uint8_t motor_index, motor_dir_e dir)
{
    /* ������� */
    if (motor_index >= CAR_MOTOR_NUM || !motor_pwm[motor_index].is_active)
    {
        return 1;
    }
    
    /* ���÷��� */
    motor_pwm[motor_index].dir = dir;
    
    /* ���µ��PWM */
    uint32_t pwm = motor_speed_to_pwm(motor_current_speed[motor_index], dir);
    uint32_t compare = pwm_us_to_compare(motor_pwm[motor_index].htim, pwm);
    __HAL_TIM_SET_COMPARE(motor_pwm[motor_index].htim, motor_pwm[motor_index].channel, compare);
    
    return 0;
}

/**
  * @brief          ���õ���ٶ�
  * @param[in]      motor_index: �������
  * @param[in]      speed: �ٶ�ֵ(-100��100)����ֵǰ������ֵ����
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_set_motor_speed(uint8_t motor_index, float speed)
{
    /* ������� */
    if (motor_index >= CAR_MOTOR_NUM || !motor_pwm[motor_index].is_active)
    {
        return 1;
    }
    
    /* �����ٶȷ�Χ */
    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < -100.0f)
    {
        speed = -100.0f;
    }
    
    /* ���µ��״̬ */
    motor_current_speed[motor_index] = speed;
    motor_target_speed[motor_index] = speed;
    
    /* ����PWMֵ */
    uint32_t pwm = motor_speed_to_pwm(speed, motor_pwm[motor_index].dir);
    uint32_t compare = pwm_us_to_compare(motor_pwm[motor_index].htim, pwm);
    
    /* ����PWM��� */
    __HAL_TIM_SET_COMPARE(motor_pwm[motor_index].htim, motor_pwm[motor_index].channel, compare);
    
    return 0;
}

/**
  * @brief          ��ȡ�����ǰ�ٶ�
  * @param[in]      motor_index: �������
  * @retval         �ٶ�ֵ(-100��100)
  */
float bsp_car_get_motor_speed(uint8_t motor_index)
{
    if (motor_index >= CAR_MOTOR_NUM)
    {
        return 0.0f;
    }
    
    return motor_current_speed[motor_index];
}

/**
  * @brief          ����С���˶�
  * @param[in]      forward_speed: ǰ���ٶ�(-100��100)
  * @param[in]      turn_rate: ת������(-100��100)����ֵ��ת����ֵ��ת
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_set_motion(float forward_speed, float turn_rate)
{
    /* �����ٶȷ�Χ */
    if (forward_speed > 100.0f)
    {
        forward_speed = 100.0f;
    }
    else if (forward_speed < -100.0f)
    {
        forward_speed = -100.0f;
    }
    
    /* ����ת��Χ */
    if (turn_rate > 100.0f)
    {
        turn_rate = 100.0f;
    }
    else if (turn_rate < -100.0f)
    {
        turn_rate = -100.0f;
    }
    
    /* �������ҵ���ٶ� */
    float left_speed = forward_speed + turn_rate;
    float right_speed = forward_speed - turn_rate;
    
    /* ��������ٶ� */
    float max_speed = fmaxf(fabsf(left_speed), fabsf(right_speed));
    if (max_speed > 100.0f)
    {
        float scale = 100.0f / max_speed;
        left_speed *= scale;
        right_speed *= scale;
    }
    
    /* ����С��״̬ */
    car_forward_speed = forward_speed;
    car_turn_rate = turn_rate;
    car_target_forward = forward_speed;
    car_target_turn = turn_rate;
    car_ctrl_mode = CAR_CTRL_MANUAL;
    
    /* ���õ���ٶ� */
    bsp_car_set_motor_speed(CAR_LEFT_MOTOR_INDEX, left_speed);
    bsp_car_set_motor_speed(CAR_RIGHT_MOTOR_INDEX, right_speed);
    
    return 0;
}

/**
  * @brief          ����С��Ŀ���˶�������ƽ������
  * @param[in]      target_forward: Ŀ��ǰ���ٶ�(-100��100)
  * @param[in]      target_turn: Ŀ��ת������(-100��100)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_set_target_motion(float target_forward, float target_turn)
{
    /* �����ٶȷ�Χ */
    if (target_forward > 100.0f)
    {
        target_forward = 100.0f;
    }
    else if (target_forward < -100.0f)
    {
        target_forward = -100.0f;
    }
    
    /* ����ת��Χ */
    if (target_turn > 100.0f)
    {
        target_turn = 100.0f;
    }
    else if (target_turn < -100.0f)
    {
        target_turn = -100.0f;
    }
    
    /* ����Ŀ��״̬ */
    car_target_forward = target_forward;
    car_target_turn = target_turn;
    car_ctrl_mode = CAR_CTRL_AUTO;
    
    return 0;
}

/**
  * @brief          С���˶����ƴ�������Ӧ��ѭ���е�����ʵ��ƽ������
  * @param[in]      none
  * @retval         none
  */
void bsp_car_motion_ctrl_update(void)
{
    /* �����Զ�����ģʽ�¹��� */
    if (car_ctrl_mode != CAR_CTRL_AUTO)
    {
        return;
    }
    
    /* ƽ������ǰ���ٶ� */
    if (car_forward_speed < car_target_forward)
    {
        car_forward_speed += CAR_SPEED_RAMP_RATE;
        if (car_forward_speed > car_target_forward)
        {
            car_forward_speed = car_target_forward;
        }
    }
    else if (car_forward_speed > car_target_forward)
    {
        car_forward_speed -= CAR_SPEED_RAMP_RATE;
        if (car_forward_speed < car_target_forward)
        {
            car_forward_speed = car_target_forward;
        }
    }
    
    /* ƽ������ת������ */
    if (car_turn_rate < car_target_turn)
    {
        car_turn_rate += CAR_SPEED_RAMP_RATE;
        if (car_turn_rate > car_target_turn)
        {
            car_turn_rate = car_target_turn;
        }
    }
    else if (car_turn_rate > car_target_turn)
    {
        car_turn_rate -= CAR_SPEED_RAMP_RATE;
        if (car_turn_rate < car_target_turn)
        {
            car_turn_rate = car_target_turn;
        }
    }
    
    /* �������ҵ���ٶ� */
    float left_speed = car_forward_speed + car_turn_rate;
    float right_speed = car_forward_speed - car_turn_rate;
    
    /* ��������ٶ� */
    float max_speed = fmaxf(fabsf(left_speed), fabsf(right_speed));
    if (max_speed > 100.0f)
    {
        float scale = 100.0f / max_speed;
        left_speed *= scale;
        right_speed *= scale;
    }
    
    /* ���µ���ٶ� */
    motor_target_speed[CAR_LEFT_MOTOR_INDEX] = left_speed;
    motor_target_speed[CAR_RIGHT_MOTOR_INDEX] = right_speed;
    
    /* ���õ��PWM */
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (!motor_pwm[i].is_active)
        {
            continue;
        }
        
        /* ƽ����������ٶ� */
        if (motor_current_speed[i] < motor_target_speed[i])
        {
            motor_current_speed[i] += CAR_SPEED_RAMP_RATE;
            if (motor_current_speed[i] > motor_target_speed[i])
            {
                motor_current_speed[i] = motor_target_speed[i];
            }
        }
        else if (motor_current_speed[i] > motor_target_speed[i])
        {
            motor_current_speed[i] -= CAR_SPEED_RAMP_RATE;
            if (motor_current_speed[i] < motor_target_speed[i])
            {
                motor_current_speed[i] = motor_target_speed[i];
            }
        }
        
        /* ����PWM��� */
        uint32_t pwm = motor_speed_to_pwm(motor_current_speed[i], motor_pwm[i].dir);
        uint32_t compare = pwm_us_to_compare(motor_pwm[i].htim, pwm);
        __HAL_TIM_SET_COMPARE(motor_pwm[i].htim, motor_pwm[i].channel, compare);
    }
}

/**
  * @brief          ң��������С��
  * @param[in]      forward_ch: ǰ��ͨ��ֵ(-660��660)
  * @param[in]      turn_ch: ת��ͨ��ֵ(-660��660)
  * @retval         ����0��ʾ�ɹ�����0��ʾʧ��
  */
uint8_t bsp_car_control_by_rc(int16_t forward_ch, int16_t turn_ch)
{
    /* Ӧ������ */
    if (forward_ch > -CAR_RC_DEADBAND && forward_ch < CAR_RC_DEADBAND)
    {
        forward_ch = 0;
    }
    if (turn_ch > -CAR_RC_DEADBAND && turn_ch < CAR_RC_DEADBAND)
    {
        turn_ch = 0;
    }
    
    /* ��ң����ֵӳ�䵽С������ֵ */
    float target_forward = (float)forward_ch / (float)CAR_RC_MAX_VALUE * 100.0f;
    float target_turn = (float)turn_ch / (float)CAR_RC_MAX_VALUE * 100.0f;
    
    /* ���ÿ���ģʽΪң�������� */
    car_ctrl_mode = CAR_CTRL_RC;
    
    /* ����Ŀ���˶� */
    return bsp_car_set_target_motion(target_forward, target_turn);
}

/**
  * @brief          ֹͣС��
  * @param[in]      none
  * @retval         none
  */
void bsp_car_stop(void)
{
    /* �������е���ٶ�Ϊ0 */
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (motor_pwm[i].is_active)
        {
            bsp_car_set_motor_speed(i, 0.0f);
        }
    }
    
    /* ����С��״̬ */
    car_forward_speed = 0.0f;
    car_turn_rate = 0.0f;
    car_target_forward = 0.0f;
    car_target_turn = 0.0f;
}

/**
  * @brief          ��ȡС����ʼ��״̬
  * @retval         ����1��ʾ�ѳ�ʼ����0��ʾδ��ʼ��
  */
uint8_t bsp_car_is_initialized(void)
{
    for (uint8_t i = 0; i < CAR_MOTOR_NUM; i++)
    {
        if (!motor_pwm[i].is_active)
        {
            return 0;
        }
    }
    
    return 1;
}