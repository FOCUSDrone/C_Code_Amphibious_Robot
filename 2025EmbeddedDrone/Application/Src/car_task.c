/**
  ******************************************************************************
  * @file       car_task.c/h
  * @brief      ·�������̣߳���Ҫ�������ߵĺ�ģ�������
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
  * @brief          car����
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
