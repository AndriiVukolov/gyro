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
#include "timers.h"
#include "stm32f4xx_it.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"
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
task_param_t par_1 = { NULL, 0, 0 };
task_param_t par_2 = { NULL, 0, 0 };
task_param_t par_3 = { " Hello from FreeRTOS ! ", 0, 3000 };
task_param_t par_4 = { NULL, 0, 1000 };
task_param_t par_5 = { NULL, 0, 2000 };
task_param_t par_6 = { NULL, 0, portMAX_DELAY };
task_param_t par_7 = { NULL, 0, portMAX_DELAY };

uint32_t t1             = 1;
uint32_t t2             = 2;
uint32_t t3             = 3;
uint32_t t4             = 4;
uint32_t timer_1_period = pdMS_TO_TICKS(1000);
uint32_t timer_2_period = pdMS_TO_TICKS(300);
uint32_t timer_3_period = pdMS_TO_TICKS(2000);
uint32_t timer_4_period = pdMS_TO_TICKS(DEBOUNCE_DELAY);

const char digits_txt[10][10] = { { "ZERO" },  { "ONE" },   { "TWO" },
                                  { "THREE" }, { "FOUR" },  { "FIVE" },
                                  { "SIX" },   { "SEVEN" }, { "EIGHT" },
                                  { "NINE" } };
const char char_msg[]         = { "control_message\r\n\0" };
const char num_msg[]          = { "1234567890\r\n\0" };

TimerHandle_t timer_1;
TimerHandle_t timer_2;
TimerHandle_t timer_3;
TimerHandle_t timer_4;

TaskHandle_t task_default;
TaskHandle_t task_1;
TaskHandle_t task_2;
TaskHandle_t task_3;
TaskHandle_t task_4;
TaskHandle_t task_5;
TaskHandle_t task_6;
TaskHandle_t task_7;

QueueHandle_t queue_1;

SemaphoreHandle_t sem_1;
SemaphoreHandle_t mutex_1;

BaseType_t success_flag = pdFAIL;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void func_char_to_uart(void *argument);
void func_num_to_uart(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void func_task_from_queue(void *argument);
void func_task_num_to_queue(void *argument);
void func_task_txt_to_queue(void *argument);
void func_button_handler(void *argument);
void func_soft_timer(TimerHandle_t x_timer);

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
    mutex_1 = xSemaphoreCreateMutex();
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    sem_1 = xSemaphoreCreateCounting(1, 0);
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    timer_1 = xTimerCreate(
            "timer_1", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_1_period,
            1,
            (void *)&t1,
            func_soft_timer);
    timer_2 = xTimerCreate(
            "timer_2", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_2_period,
            0,
            (void *)&t2,
            func_soft_timer);
    timer_3 = xTimerCreate(
            "timer_3", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_3_period,
            0,
            (void *)&t3,
            func_soft_timer);
    timer_4 = xTimerCreate(
            "timer_4", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_4_period,
            0,
            (void *)&t4,
            func_soft_timer);
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    queue_1 = xQueueCreate(5, sizeof(queue_element_t));
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    xTaskCreate(StartDefaultTask,
                "task_default",
                configMINIMAL_STACK_SIZE,
                NULL,
                osPriorityIdle,
                &task_default);
    xTaskCreate(func_button_handler,
                "task_2",
                configMINIMAL_STACK_SIZE,
                (void *)&par_2,
                osPriorityBelowNormal1,
                &task_2);
    //    xTaskCreate(func_task_from_queue,
    //                "task_3",
    //                configMINIMAL_STACK_SIZE,
    //                (void *)&par_3,
    //                osPriorityLow3,
    //                &task_3);
    //    xTaskCreate(func_task_num_to_queue,
    //                "task_4",
    //                configMINIMAL_STACK_SIZE,
    //                (void *)&par_4,
    //                osPriorityLow2,
    //                &task_4);
    //    xTaskCreate(func_task_txt_to_queue,
    //                "task_5",
    //                configMINIMAL_STACK_SIZE,
    //                (void *)&par_5,
    //                osPriorityLow2,
    //                &task_5);
    xTaskCreate(func_char_to_uart,
                "task_6",
                configMINIMAL_STACK_SIZE,
                (void *)&par_6,
                osPriorityLow1,
                &task_6);
    xTaskCreate(func_num_to_uart,
                "task_7",
                configMINIMAL_STACK_SIZE,
                (void *)&par_7,
                osPriorityLow1,
                &task_7);

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    success_flag = xTimerStart(timer_1, 0);
    if (success_flag != pdPASS)
        print("Unable to start timer_1");
    success_flag = xTimerStart(timer_2, 0);
    if (success_flag != pdPASS)
        print("Unable to start timer_2");
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

/* USER CODE BEGIN Header_func_task_from_queue */
/**
* @brief Function implementing the task_3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_task_from_queue */
void func_task_from_queue(void *argument)
{
    /* USER CODE BEGIN func_task_from_queue */
    BaseType_t      op_status = pdFAIL;
    task_param_t   *ptr_str;
    queue_element_t q2;
    char            str_buf[50] = { 0 };

    ptr_str = (task_param_t *)argument;

    sprintf(str_buf, "%s \r\n", ptr_str->str);
    print(str_buf);
    /* Infinite loop */
    for (;;) {
        op_status = xQueueReceive(queue_1, &q2, portMAX_DELAY);
        if (op_status == pdPASS) {
            if (q2.src == TASK_SEND_NUMBER)
                sprintf(str_buf, "%lu \r\n", q2.nmb);
            else {
                if (q2.nmb == 11)
                    sprintf(str_buf, "BUTTON IS PRESSED \r\n");
                else
                    sprintf(str_buf, "%s \r\n", digits_txt[q2.nmb]);
            }
            print(str_buf);
        } else
            print("The queue is empty! \r\n");
        //portYIELD();
    }
    /* USER CODE END func_task_from_queue */
}

/* USER CODE BEGIN Header_func_task_num_to_queue */
/**
* @brief Function implementing the task_4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_task_num_to_queue */
void func_task_num_to_queue(void *argument)
{
    /* USER CODE BEGIN func_task_num_to_queue */
    BaseType_t             op_status = pdFAIL;
    task_param_t          *ptr_str;
    static queue_element_t q_el_num = { 0, TASK_SEND_NUMBER };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->period);
    /* Infinite loop */
    for (;;) {
        q_el_num.nmb++;
        op_status = xQueueSend(queue_1, &q_el_num, ticks_wait);
        if (op_status != pdPASS)
            print("Can`t send number to queue \r\n");

        vTaskDelay(ticks_wait);
    }
    /* USER CODE END func_task_num_to_queue */
}

/* USER CODE BEGIN Header_func_task_txt_to_queue */
/**
* @brief Function implementing the task_5 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func_task_txt_to_queue */
void func_task_txt_to_queue(void *argument)
{
    /* USER CODE BEGIN func_task_txt_to_queue */
    BaseType_t             op_status = pdFAIL;
    task_param_t          *ptr_str;
    static queue_element_t q_el_txt = { 0, TASK_SEND_TEXT };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->period);
    /* Infinite loop */
    for (;;) {
        if (q_el_txt.nmb < 9)
            q_el_txt.nmb++;
        else
            q_el_txt.nmb = 0;
        //op_status = xQueueSend(queue1Handle, &q_el_txt, ticks_wait);
        op_status = xQueueSend(queue_1, &q_el_txt, ticks_wait);
        if (op_status != pdPASS)
            print("Can`t send text to queue \r\n");

        vTaskDelay(ticks_wait);
    }
    /* USER CODE END func_task_txt_to_queue */
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
    BaseType_t      op_status = pdFAIL;
    task_param_t   *ptr_str;
    queue_element_t q_el_evt = { 11, TASK_SEND_TEXT };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->period);
    /* Infinite loop */
    while (1) {
        xSemaphoreTake(sem_1, portMAX_DELAY);
        op_status = xQueueSend(queue_1, &q_el_evt, ticks_wait);
        if (op_status != pdPASS)
            print("Can`t send message to queue \r\n");
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
    uint32_t *timer_id;
    timer_id = (uint32_t *)pvTimerGetTimerID(x_timer);

    if (*timer_id == 1) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    } else if (*timer_id == 2) {
        xTimerStart(timer_3, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    } else if (*timer_id == 3) {
        xTimerStart(timer_2, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    } else if (*timer_id == 4) {
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET)
            xSemaphoreGive(sem_1);
        else
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
        xTimerStartFromISR(timer_4, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void func_char_to_uart(void *argument)
{
    const char *c = NULL;
    c             = char_msg;

    while (1) {
        if (xSemaphoreTake(mutex_1, portMAX_DELAY) == pdTRUE) {
            while (*c != '\0') {
                HAL_UART_Transmit(&huart1, (const uint8_t *)c, 1, 10);
                c++;
                HAL_Delay(100);
            }
            xSemaphoreGive(mutex_1);
        }
    }
}
void func_num_to_uart(void *argument)
{
    const char *c = NULL;
    c             = num_msg;

    while (1) {
        if (xSemaphoreTake(mutex_1, portMAX_DELAY) == pdTRUE) {
            while (*c != '\0') {
                HAL_UART_Transmit(&huart1, (const uint8_t *)c, 1, 10);
                c++;
                HAL_Delay(100);
            }
            xSemaphoreGive(mutex_1);
        }
    }
}

/* USER CODE END Application */
