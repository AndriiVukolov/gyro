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
#include "timers.h"
#include "stm32f4xx_it.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include "gyro_I3G4250D.h"
#include "accel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY     200
#define OPTIMAL_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint32_t t_gyro   = 1;
uint32_t t_accel  = 2;
uint32_t t_data   = 3;
uint32_t t_button = 4;

uint32_t timer_gyro_poll_period  = pdMS_TO_TICKS(500);
uint32_t timer_accel_poll_period = pdMS_TO_TICKS(500);
uint32_t timer_data_show_period  = pdMS_TO_TICKS(1000);
uint32_t timer_gyro_read_period  = pdMS_TO_TICKS(500);
uint32_t timer_print_period      = pdMS_TO_TICKS(1000);
uint32_t timer_debounce_period   = pdMS_TO_TICKS(DEBOUNCE_DELAY);

task_param_t parameter_gyro      = { 0 };
task_param_t parameter_accel     = { 0 };
task_param_t parameter_gyro_read = { 0 };
task_param_t parameter_print     = { 0 };

TimerHandle_t timer_gyro_poll;
TimerHandle_t timer_accel_poll;
TimerHandle_t timer_data_show; //user
TimerHandle_t timer_button_debounce;

TaskHandle_t task_default;
TaskHandle_t task_gyro_handler;
TaskHandle_t task_accel_handler;
TaskHandle_t task_cli_handler;
TaskHandle_t task_print;

QueueHandle_t queue_gyro;
QueueHandle_t queue_accel;
QueueHandle_t queue_cli_command;
QueueHandle_t queue_print;

SemaphoreHandle_t sem_timer_gyro;
SemaphoreHandle_t sem_timer_accel;
SemaphoreHandle_t sem_data_show;
SemaphoreHandle_t sem_button_debounce;
SemaphoreHandle_t mutex_spi;
SemaphoreHandle_t mutex_uart;

BaseType_t success_flag = pdFAIL;

enum timer_mode { ONCE_COUNT, AUTORELOAD };

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void func_gyro_poll(void *argument);
void func_accel_poll(void *argument);
void func_cli_handler(void *argument);
void func_read_gyro_queue(void *argument);
void StartDefaultTask(void *argument);
void func_task_from_queue(void *argument);
void func_button_handler(void *argument);
void func_soft_timer(TimerHandle_t x_timer);
void func_print(void *argument);
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
    parameter_gyro.handler     = (void *)&g1;
    parameter_gyro.poll_period = timer_gyro_poll_period;

    parameter_accel.handler     = (void *)&a1;
    parameter_accel.poll_period = timer_accel_poll_period;

    parameter_gyro_read.handler     = (void *)&g1;
    parameter_gyro_read.poll_period = timer_gyro_read_period;

    parameter_print.handler     = NULL;
    parameter_print.poll_period = timer_print_period;
    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    mutex_spi  = xSemaphoreCreateMutex();
    mutex_uart = xSemaphoreCreateMutex();
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    //    sem_timer_gyro      = xSemaphoreCreateBinary();
    //    sem_timer_accel     = xSemaphoreCreateBinary();
    //    sem_button_debounce = xSemaphoreCreateBinary();
    //    sem_data_show       = xSemaphoreCreateBinary();
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    //    timer_gyro_poll       = xTimerCreate("timer_gyro_poll",
    //                                   timer_gyro_poll_period,
    //                                   AUTORELOAD,
    //                                   (void *)&t_gyro,
    //                                   func_soft_timer);
    //    timer_accel_poll      = xTimerCreate("timer_accel_poll",
    //                                    timer_accel_poll_period,
    //                                    AUTORELOAD,
    //                                    (void *)&t_accel,
    //                                    func_soft_timer);
    //    timer_data_show       = xTimerCreate("timer_data_show",
    //                                   timer_data_show_period,
    //                                   AUTORELOAD,
    //                                   (void *)&t_data,
    //                                   func_soft_timer);
    timer_button_debounce = xTimerCreate("timer_button_debounce",
                                         timer_debounce_period,
                                         ONCE_COUNT,
                                         (void *)&t_button,
                                         func_soft_timer);
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    queue_gyro  = xQueueCreate(10, sizeof(queue_data_element_t));
    queue_accel = xQueueCreate(10, sizeof(queue_data_element_t));
    queue_print = xQueueCreate(4, sizeof(char[100]));

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
    success_flag = xTaskCreate(func_gyro_poll,
                               "task_gyro_handler",
                               OPTIMAL_STACK_SIZE,
                               (void *)&parameter_gyro,
                               osPriorityBelowNormal1,
                               &task_gyro_handler);
    //    success_flag = xTaskCreate(func_accel_poll,
    //                "task_accel_handler",
    //                configMINIMAL_STACK_SIZE,
    //                (void *)&parameter_accel,
    //                osPriorityBelowNormal1,
    //                &task_accel_handler);
    //    success_flag = xTaskCreate(func_cli_handler,
    //                "task_cli_handler",
    //                configMINIMAL_STACK_SIZE,
    //                NULL,
    //                osPriorityBelowNormal1,
    //                &task_cli_handler);
    success_flag = xTaskCreate(func_read_gyro_queue,
                               "task_read_gyro_queue",
                               OPTIMAL_STACK_SIZE,
                               (void *)&parameter_gyro_read,
                               osPriorityBelowNormal2,
                               &task_cli_handler);
    success_flag = xTaskCreate(func_print,
                               "task_print",
                               OPTIMAL_STACK_SIZE,
                               (void *)&parameter_print,
                               osPriorityBelowNormal1,
                               &task_print);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    //    success_flag = xTimerStart(timer_gyro_poll, portMAX_DELAY);
    //    if (success_flag != pdPASS)
    //        print("Unable to start timer_gyro_poll");
    //    success_flag = xSemaphoreTake(sem_timer_gyro, portMAX_DELAY);
    //    if ((success_flag != pdPASS) || (sem_timer_gyro == NULL))
    //        print("Unable to take a sem_timer_gyro");

    //    success_flag = xTimerStart(timer_accel_poll, portMAX_DELAY);
    //    if (success_flag != pdPASS)
    //        print("Unable to start timer_accel_poll");
    //    success_flag = xSemaphoreTake(sem_timer_accel, portMAX_DELAY);
    //    if ((success_flag != pdPASS) || (sem_timer_accel == NULL))
    //        print("Unable to take a sem_timer_accel");

    //    success_flag = xTimerStart(timer_data_show, portMAX_DELAY);
    //    if (success_flag != pdPASS)
    //        print("Unable to start timer_data_show");
    //    success_flag = xSemaphoreTake(sem_data_show, portMAX_DELAY);
    //    if ((success_flag != pdPASS) || (sem_data_show == NULL))
    //        print("Unable to take a sem_data_show");
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
    /* USER CODE BEGIN func_soft_timer */
    //uint32_t timer_number = *(uint32_t *)argument;
    //    osTimerId_t timer_id;
    //    uint32_t *timer_id;
    //    timer_id = (uint32_t *)pvTimerGetTimerID(x_timer);

    //    if (x_timer == timer_gyro_poll) {
    //        //xSemaphoreGive(sem_timer_gyro);
    //
    //        //HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    //
    //    } else if (x_timer == timer_accel_poll) {
    //        //xSemaphoreGive(sem_timer_accel);
    //
    //    } else if (x_timer == timer_data_show) {
    //        //xSemaphoreGive(sem_data_show);

    /*} else*/ if (x_timer == timer_button_debounce) {
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

void func_gyro_poll(void *argument)
{
    task_param_t        *pr           = NULL;
    float                vals[3]      = { 0 };
    gyroError_t          err          = gyroOk;
    queue_data_element_t qg           = { 0 };
    queue_data_element_t dummy        = { 0 };
    BaseType_t           op_status    = pdFAIL;
    char                 str_buf[100] = { 0 };

    pr = (task_param_t *)argument;
    while (1) {
        //xSemaphoreTake(sem_timer_gyro, portMAX_DELAY);
        xSemaphoreTake(mutex_spi, 2);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        err = gyroReadVal(pr->handler, vals);

        if (err == gyroOk) {
            qg.val_x     = vals[0];
            qg.val_y     = vals[1];
            qg.val_z     = vals[2];
            qg.timestamp = xTaskGetTickCount(); //ms
            op_status    = xQueueSend(queue_gyro, &qg, pr->poll_period);
            if (op_status != pdPASS) {
                op_status = xQueueReceive(queue_gyro, &dummy, 0);
                op_status = xQueueSend(queue_gyro, &qg, pr->poll_period);
            }
        } else {
            strcpy(str_buf, "SPI problem \r\n");
        }
        op_status = xQueueSend(queue_print, str_buf, 0);
        xSemaphoreGive(mutex_spi);
        //vTaskDelay(pr->poll_period);
        //portYIELD();
    }
}

void func_accel_poll(void *argument)
{
    task_param_t        *pr           = NULL;
    float                vals[3]      = { 0 };
    accel_error_t        err          = ACCEL_OK;
    queue_data_element_t qg           = { 0 };
    queue_data_element_t dummy        = { 0 };
    BaseType_t           op_status    = pdFAIL;
    char                 str_buf[100] = { 0 };

    pr = (task_param_t *)argument;
    while (1) {
        //xSemaphoreTake(sem_timer_accel, portMAX_DELAY);

        xSemaphoreTake(mutex_spi, 2);
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        err = accel_data_get(pr->handler, vals);

        if (err == ACCEL_OK) {
            qg.val_x     = vals[0];
            qg.val_y     = vals[1];
            qg.val_z     = vals[2];
            qg.timestamp = xTaskGetTickCount(); //ms
            op_status    = xQueueSend(queue_accel, &qg, pr->poll_period);
            if (op_status != pdPASS) {
                op_status = xQueueReceive(queue_accel, &dummy, 0);
                op_status = xQueueSend(queue_accel, &qg, pr->poll_period);
            }
        } else
            strcpy(str_buf, "SPI problem \r\n");
        op_status = xQueueSend(queue_print, str_buf, 0);
        xSemaphoreGive(mutex_spi);
        //portYIELD();
        //vTaskDelay(pr->poll_period);
    }
}
void func_read_gyro_queue(void *argument)
{
    /* USER CODE BEGIN func_task_from_queue */
    task_param_t        *pr           = NULL;
    BaseType_t           op_status    = pdFAIL;
    queue_data_element_t gqel         = { 0 };
    char                 str_buf[100] = { 0 };
    pr                                = (task_param_t *)argument;

    /* Infinite loop */
    while (1) {
        //xSemaphoreTake(sem_data_show, portMAX_DELAY);
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        op_status = xQueueReceive(queue_gyro, &gqel, pr->poll_period);
        if (op_status == pdPASS) {
            sprintf(str_buf,
                    "X=%f, Y=%f, Z=%f, Timestamp=%lu; \r\n",
                    gqel.val_x,
                    gqel.val_y,
                    gqel.val_z,
                    gqel.timestamp);
        } else {
            strcpy(str_buf, "The queue is empty! \r\n");
        }
        if (op_status != pdPASS)
            strcpy(str_buf, "The queue to print is full! \r\n");
        op_status = xQueueSend(queue_print, str_buf, pr->poll_period);
    }

    //vTaskDelay(pr->poll_period);
    //portYIELD();
}

void func_print(void *argument)
{
    task_param_t *pr        = NULL;
    char          pel[100]  = { 0 };
    BaseType_t    op_status = pdFAIL;

    pr = (task_param_t *)argument;

    while (1) {
        //xSemaphoreTake(mutex_uart, pr->poll_period);
        op_status = xQueueReceive(queue_print, &pel, pr->poll_period);
        if (op_status == pdPASS)
            print(pel);
        else
            print("Failed to print!");
        //xSemaphoreGive(mutex_uart);
    }
    vTaskDelay(pr->poll_period);
}
/* USER CODE END Application */
