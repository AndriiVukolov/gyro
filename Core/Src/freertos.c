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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void StartDefaultTask(void *argument);

void func_button_handler(void *argument);
void func_soft_timer(TimerHandle_t x_timer);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
    //    sem_timer_gyro      = xSemaphoreCreateBinary();
    //    sem_timer_accel     = xSemaphoreCreateBinary();
    sem_button_debounce = xSemaphoreCreateBinary();

    //    sem_data_show       = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    timer_button_debounce = xTimerCreate("timer_button_debounce",
                                         timer_debounce_period,
                                         ONCE_COUNT,
                                         (void *)&t_button,
                                         func_soft_timer);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
/* USER CODE END Application */

