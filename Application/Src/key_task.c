#include "key_task.h"
#include "main.h"
#include "cmsis_os.h"

static key_data_t key_data;

static void key_data_init()
{
    key_data.state = key_data.last_state = KEY_RELEASE;
    key_data.state_change_cnt = key_data.press_cnt = key_data.release_cnt = \
    key_data.short_press_cnt = key_data.long_press_cnt =  0;
    
}

const key_data_t *get_key_data_point(void){
    return &key_data;
}

/**
  * @brief          key task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          key任务，由于中断0被姿态解算任务用于唤醒线程，故按键只能采用轮询方法
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void key_task(void const * argument)
{
	key_data_init();
    while(1)
    {
		if(key_data.state == KEY_RELEASE){
            
            if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(KEY_JITTER_TIME);
                if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET){
                    key_data.last_state = key_data.state;
                    key_data.state = KEY_PRESS;
                    key_data.press_time = xTaskGetTickCount();
                    key_data.state_change_cnt++;
                    key_data.press_cnt++;
                }
            }
        }
        else if(key_data.state == KEY_PRESS){
            if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                vTaskDelay(KEY_JITTER_TIME);
                if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET){
                    key_data.last_state = key_data.state;
                    key_data.state = KEY_RELEASE;
                    key_data.release_time = xTaskGetTickCount();
                    key_data.state_change_cnt++;
                    key_data.release_cnt++;
                    
                    if(key_data.release_time - key_data.press_time > KEY_LONG_PRESS_TIME){
                        key_data.long_press_cnt++;
                    }
                    else{
                        key_data.short_press_cnt++;
                    }
                }
            }
        }
        vTaskDelay(KEY_SCAN_TIME);
    }
}
