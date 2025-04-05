/**
 * @file test_task.c
 * @brief 用于测试风扇控制的任务
 * @author Feiziben
 * @date 2025.03.20
 */
#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "key_task.h"
#include "bsp_fan.h"
#include "bsp_push_rog.h"

/* 按键输入数据 */
const key_data_t *key_data_point;
uint8_t test_short_press_cnt;
uint8_t test_long_press_cnt;
uint8_t fan_state = 0;
static bool_t key_flip_state;

/**
 * @brief 测试任务
 * @param[in] argument 未使用
 * @retval none
 */
void test_task(void const * argument)
{
    /* 初始化按键数据和计数器 */
    key_data_point = get_key_data_point();
    test_short_press_cnt = 0;
    test_long_press_cnt = 0;
    
    /* 初始化风扇 */
    fan_init();
    
#ifndef TEST_TASK_WORK
    while(1) {
        vTaskDelay(TEST_TASK_TIME);
    }
#endif

    while(1) {
        
        
        /* 长按：控制推杆 */
        if(key_data_point->long_press_cnt > test_long_press_cnt) {
            test_long_press_cnt = key_data_point->long_press_cnt;
            
            if (key_flip_state == 0) {
                elongate_left_push_rog();
                elongate_right_push_rog();
                key_flip_state = 1;
            } 
            else {
                shorten_left_push_rog();
                shorten_right_push_rog();
                key_flip_state = 0;
            }
        }
        
        /* 调用风扇处理函数 */
        fan_process();
        
        /* 任务延时 */
        vTaskDelay(TEST_TASK_TIME);
    }
}


