#ifndef BSP_PUSH_ROG_H
#define BSP_PUSH_ROG_H
#include "struct_typedef.h"

#define LEFT_PUSH_ROG_IN1_GPIO  GPIOI
#define LEFT_PUSH_ROG_IN1_PIN   GPIO_PIN_6
#define LEFT_PUSH_ROG_IN2_GPIO  GPIOI
#define LEFT_PUSH_ROG_IN2_PIN   GPIO_PIN_7

#define RIGHT_PUSH_ROG_IN3_GPIO GPIOE
#define RIGHT_PUSH_ROG_IN3_PIN  GPIO_PIN_13
#define RIGHT_PUSH_ROG_IN4_GPIO GPIOE
#define RIGHT_PUSH_ROG_IN4_PIN  GPIO_PIN_14

extern void left_push_rog_off(void);
extern void right_push_rog_off(void);
extern void elongate_left_push_rog(void);
extern void shorten_left_push_rog(void);
extern void elongate_right_push_rog(void);
extern void shorten_right_push_rog(void);

#endif
