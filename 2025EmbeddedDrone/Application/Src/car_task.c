/**
  ******************************************************************************
  * @file       car_task.c/h
  * @brief      路地行走线程，主要负责行走的航模电机驱动
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
  
#include "car_task.h"
#include "main.h"
#include "cmsis_os.h"


/**
  * @brief          car任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void car_task(void const * argument)
{
    
#ifdef TEST_TASK_WORK
    while (1)
    {
        vTaskDelay(CAR_TASK_TIME);
    }
#endif
    
    while(1)
    {
        
        
        vTaskDelay(CAR_TASK_TIME);
    }
}
