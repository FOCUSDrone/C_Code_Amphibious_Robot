#include "bsp_push_rog.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

void left_push_rog_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PUSH_ROG_OFF_PWM);
    HAL_GPIO_WritePin(LEFT_PUSH_ROG_GPIO, LEFT_PUSH_ROG_PIN, GPIO_PIN_RESET);
}

void elongate_left_push_rog(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PUSH_ROG_ON_PWM);
    HAL_GPIO_WritePin(LEFT_PUSH_ROG_GPIO, LEFT_PUSH_ROG_PIN, GPIO_PIN_SET);
}

void shorten_left_push_rog(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PUSH_ROG_ON_PWM);
    HAL_GPIO_WritePin(LEFT_PUSH_ROG_GPIO, LEFT_PUSH_ROG_PIN, GPIO_PIN_RESET);
}

void right_push_rog_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PUSH_ROG_OFF_PWM);
    HAL_GPIO_WritePin(RIGHT_PUSH_ROG_GPIO, RIGHT_PUSH_ROG_PIN, GPIO_PIN_RESET);
}

void elongate_right_push_rog(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PUSH_ROG_ON_PWM);
    HAL_GPIO_WritePin(RIGHT_PUSH_ROG_GPIO, RIGHT_PUSH_ROG_PIN, GPIO_PIN_SET);
}

void shorten_right_push_rog(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PUSH_ROG_ON_PWM);
    HAL_GPIO_WritePin(RIGHT_PUSH_ROG_GPIO, RIGHT_PUSH_ROG_PIN, GPIO_PIN_RESET);
}
