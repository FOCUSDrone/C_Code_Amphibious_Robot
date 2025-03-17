#include "bsp_adsorption.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;

void adsorp_on(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);

}

void adsorp_off(void)
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
}
