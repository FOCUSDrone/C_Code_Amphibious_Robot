#ifndef BSP_CAR_H
#define BSP_CAR_H
#include "struct_typedef.h"

void car_left_motor_on(uint16_t pwm);
void car_left_motor_off(void);
void car_right_motor_on(uint16_t pwm);
void car_right_motor_off(void);

#endif
