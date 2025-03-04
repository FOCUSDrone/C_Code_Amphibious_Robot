#ifndef BSP_PUSH_ROG_H
#define BSP_PUSH_ROG_H
#include "struct_typedef.h"

#define PUSH_ROG_ON_PWM     1500.0f
#define PUSH_ROG_OFF_PWM    0.0f

#define LEFT_PUSH_ROG_GPIO  GPIOI
#define LEFT_PUSH_ROG_PIN   GPIO_PIN_6
#define RIGHT_PUSH_ROG_GPIO GPIOI
#define RIGHT_PUSH_ROG_PIN  GPIO_PIN_7    


extern void left_push_rog_off(void);
extern void right_push_rog_off(void);
extern void elongate_left_push_rog(void);
extern void shorten_left_push_rog(void);
extern void elongate_right_push_rog(void);
extern void shorten_right_push_rog(void);
#endif
