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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_it.h"
#include "semphr.h"
#include "usart.h"
#include "sensor_service.h"
#include "log_service.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t          t_button              = 4;
uint32_t          timer_debounce_period = pdMS_TO_TICKS(DEBOUNCE_DELAY);
TimerHandle_t     timer_button_debounce;
TaskHandle_t      task_default;
SemaphoreHandle_t sem_button_debounce;
BaseType_t        success_flag = pdFAIL;

enum timer_mode { ONCE_COUNT, AUTORELOAD };

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void StartDefaultTask(void *argument);

void func_button_handler(void *argument);
void func_soft_timer(TimerHandle_t x_timer);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}

/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    //    sem_timer_gyro      = xSemaphoreCreateBinary();
    //    sem_timer_accel     = xSemaphoreCreateBinary();
    sem_button_debounce = xSemaphoreCreateBinary();

    //    sem_data_show       = xSemaphoreCreateBinary();

    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    timer_button_debounce = xTimerCreate("timer_button_debounce",
                                         timer_debounce_period,
                                         ONCE_COUNT,
                                         (void *)&t_button,
                                         func_soft_timer);
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    success_flag = xTaskCreate(StartDefaultTask,
                               "task_default",
                               configMINIMAL_STACK_SIZE,
                               NULL,
                               osPriorityIdle,
                               &task_default);
    success_flag &= ~sensor_poll_start(SENSOR_ENABLE, SENSOR_ENABLE, 10);
    success_flag &= read_log_service_start();

    if (success_flag == pdFAIL)
        print("SYSTEM FAIL: unable to create tasks! \r\n");

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */

    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        vTaskDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_func_button_handler */
/**
* @brief Function implementing the task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_button_handler */
void func_button_handler(void *argument)
{
    /* USER CODE BEGIN func_button_handler */
    BaseType_t op_status = pdFAIL;
    /* Infinite loop */
    while (1) {
        if (xSemaphoreTake(sem_button_debounce, portMAX_DELAY) == pdPASS) {
            print("Button pressed \r\n");
            if (op_status != pdPASS)
                print("Can`t send message to queue \r\n");
        }
        HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);
    }
    /* USER CODE END func_button_handler */
}

/* func_soft_timer function */
void func_soft_timer(TimerHandle_t x_timer)
{
    if (x_timer == timer_button_debounce) {
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET)
            xSemaphoreGive(sem_button_debounce);
        HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);
    }

    /* USER CODE END func_soft_timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == BUTTON_Pin) {
        HAL_NVIC_DisableIRQ(BUTTON_EXTI_IRQn);
        xTimerStartFromISR(timer_button_debounce, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* USER CODE END Application */
