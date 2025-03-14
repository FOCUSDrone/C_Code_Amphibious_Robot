#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "struct_typedef.h"
#include "cmsis_os.h"

/* ������ز��� */
#define CAR_TASK_INIT_TIME    500    // �����ʼ���ȴ�ʱ�䣬��λms
#define CAR_CONTROL_TIME_MS   2      // �������ڣ���λms

/**
  * @brief          С��������RTOS��ע��Ϊ�߳�
  * @param[in]      pvParameters: �̲߳���
  * @retval         none
  */
extern void car_task(void *pvParameters);

/**
  * @brief          ����С�������߳�
  * @param[in]      none
  * @retval         none
  */
extern void car_task_create(void);

#endif

