#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "struct_typedef.h"
#include "cmsis_os.h"

/* 任务相关参数 */
#define CAR_TASK_INIT_TIME    500    // 任务初始化等待时间，单位ms
#define CAR_CONTROL_TIME_MS   2      // 控制周期，单位ms

/**
  * @brief          小车任务，在RTOS中注册为线程
  * @param[in]      pvParameters: 线程参数
  * @retval         none
  */
extern void car_task(void *pvParameters);

/**
  * @brief          创建小车任务线程
  * @param[in]      none
  * @retval         none
  */
extern void car_task_create(void);

#endif

