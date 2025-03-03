#ifndef KEY_TASK_H
#define KEY_TASK_H
#include "struct_typedef.h"

#define KEY_SCAN_TIME       1
#define KEY_JITTER_TIME     20

#define KEY_Pin         GPIO_PIN_0
#define KEY_GPIO_Port   GPIOA

#define KEY_PRESS       0
#define KEY_RELEASE     1

#define KEY_LONG_PRESS_TIME 1500

typedef struct
{
    bool_t state;
    bool_t last_state;
    uint8_t state_change_cnt;
    uint8_t press_cnt;
    uint8_t release_cnt;
    
    uint32_t press_time;
    uint32_t release_time;
    uint8_t long_press_cnt;
    uint8_t short_press_cnt;
} key_data_t;

extern const key_data_t *get_key_data_point(void);

#endif
