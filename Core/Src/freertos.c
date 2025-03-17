/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId imuTaskHandle;
osThreadId testTaskHandle;
osThreadId buzzerTaskHandle;
osThreadId ledTaskHandle;
osThreadId usbTaskHandle;
osThreadId detectTaskHandle;
osThreadId keyTaskHandle;
osThreadId transformTaskHandle;
osThreadId carTaskHandle;
osThreadId servoRxTaskHandle;
osThreadId adsorptionTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_task(void const * argument);
void test_task(void const * argument);
void buzzer_task(void const * argument);
void led_task(void const * argument);
void usb_task(void const * argument);
void detect_task(void const * argument);
void key_task(void const * argument);
void transform_task(void const * argument);
void car_task(void const * argument);
void servo_receive_task(void const * argument);
void adsorption_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of imuTask */
  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of testTask */
  osThreadDef(testTask, test_task, osPriorityNormal, 0, 256);
  testTaskHandle = osThreadCreate(osThread(testTask), NULL);

  /* definition and creation of buzzerTask */
  osThreadDef(buzzerTask, buzzer_task, osPriorityNormal, 0, 128);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, led_task, osPriorityNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, usb_task, osPriorityNormal, 0, 128);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* definition and creation of detectTask */
  osThreadDef(detectTask, detect_task, osPriorityNormal, 0, 256);
  detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

  /* definition and creation of keyTask */
  osThreadDef(keyTask, key_task, osPriorityNormal, 0, 128);
  keyTaskHandle = osThreadCreate(osThread(keyTask), NULL);

  /* definition and creation of transformTask */
  osThreadDef(transformTask, transform_task, osPriorityNormal, 0, 256);
  transformTaskHandle = osThreadCreate(osThread(transformTask), NULL);

  /* definition and creation of carTask */
  osThreadDef(carTask, car_task, osPriorityNormal, 0, 256);
  carTaskHandle = osThreadCreate(osThread(carTask), NULL);

  /* definition and creation of servoRxTask */
  osThreadDef(servoRxTask, servo_receive_task, osPriorityNormal, 0, 128);
  servoRxTaskHandle = osThreadCreate(osThread(servoRxTask), NULL);

  /* definition and creation of adsorptionTask */
  osThreadDef(adsorptionTask, adsorption_task, osPriorityNormal, 0, 256);
  adsorptionTaskHandle = osThreadCreate(osThread(adsorptionTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_task */
/**
  * @brief  Function implementing the imuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_test_task */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_buzzer_task */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzer_task */
__weak void buzzer_task(void const * argument)
{
  /* USER CODE BEGIN buzzer_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END buzzer_task */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
__weak void led_task(void const * argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_usb_task */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_task */
__weak void usb_task(void const * argument)
{
  /* USER CODE BEGIN usb_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usb_task */
}

/* USER CODE BEGIN Header_detect_task */
/**
* @brief Function implementing the detectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_task */
__weak void detect_task(void const * argument)
{
  /* USER CODE BEGIN detect_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END detect_task */
}

/* USER CODE BEGIN Header_key_task */
/**
* @brief Function implementing the keyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_key_task */
__weak void key_task(void const * argument)
{
  /* USER CODE BEGIN key_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END key_task */
}

/* USER CODE BEGIN Header_transform_task */
/**
* @brief Function implementing the transformTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transform_task */
__weak void transform_task(void const * argument)
{
  /* USER CODE BEGIN transform_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END transform_task */
}

/* USER CODE BEGIN Header_car_task */
/**
* @brief Function implementing the carTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_car_task */
__weak void car_task(void const * argument)
{
  /* USER CODE BEGIN car_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END car_task */
}

/* USER CODE BEGIN Header_servo_receive_task */
/**
* @brief Function implementing the servoRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_receive_task */
__weak void servo_receive_task(void const * argument)
{
  /* USER CODE BEGIN servo_receive_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servo_receive_task */
}

/* USER CODE BEGIN Header_adsorption_task */
/**
* @brief Function implementing the adsorptionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adsorption_task */
__weak void adsorption_task(void const * argument)
{
  /* USER CODE BEGIN adsorption_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END adsorption_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
