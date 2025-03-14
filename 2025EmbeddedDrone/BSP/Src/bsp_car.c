#include "bsp_car.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

void car_left_motor_on(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);

}

void car_left_motor_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
}

void car_right_motor_on(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);

}

void car_right_motor_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
}
