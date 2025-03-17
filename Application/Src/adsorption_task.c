/**
  ******************************************************************************
  * @file       adsorption_task.c/h
  * @brief      吸附线程
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
  
#include "adsorption_task.h"
#include "cmsis_os.h"
#include "remote_receive.h"
#include "arm_math.h"
#include "user_lib.h"

/**
  * @brief          adsorption任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void adsorption_task(void const * argument)
{
#ifdef TEST_TASK_WORK
    while (1)
    {
        vTaskDelay(ADSORPTION_TASK_TIME);
    }
#endif
    
    while(1)
    {
        
        vTaskDelay(ADSORPTION_TASK_TIME);
    }
}