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
#include "stdio.h"
#include "timers.h"
#include "stm32f4xx_it.h"
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

TimerHandle_t timer_1Handle;
TimerHandle_t timer_2Handle;
TimerHandle_t timer_3Handle;
TimerHandle_t timer_4Handle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t         defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityIdle,
};
/* Definitions for task_3 */
osThreadId_t         task_3Handle;
const osThreadAttr_t task_3_attributes = {
    .name       = "task_3",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow3,
};
/* Definitions for task_4 */
osThreadId_t         task_4Handle;
const osThreadAttr_t task_4_attributes = {
    .name       = "task_4",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow2,
};
/* Definitions for task_5 */
osThreadId_t         task_5Handle;
const osThreadAttr_t task_5_attributes = {
    .name       = "task_5",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow2,
};
/* Definitions for task_2 */
osThreadId_t         task_2Handle;
const osThreadAttr_t task_2_attributes = {
    .name       = "task_2",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal1,
};
/* Definitions for queue1 */
osMessageQueueId_t         queue1Handle;
const osMessageQueueAttr_t queue1_attributes = { .name = "queue1" };
/* Definitions for timer_1 */
//osTimerId_t         timer_1Handle;
//const osTimerAttr_t timer_1_attributes = { .name = "timer_1" };
///* Definitions for timer_2 */
//osTimerId_t         timer_2Handle;
//const osTimerAttr_t timer_2_attributes = { .name = "timer_2" };
///* Definitions for timer_3 */
//osTimerId_t         timer_3Handle;
//const osTimerAttr_t timer_3_attributes = { .name = "timer_3" };
///* Definitions for timer_4 */
//osTimerId_t         timer_4Handle;
//const osTimerAttr_t timer_4_attributes = { .name = "timer_4" };
/* Definitions for sem_1_binary */
osSemaphoreId_t         sem_1_binaryHandle;
const osSemaphoreAttr_t sem_1_binary_attributes = { .name = "sem_1_binary" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of sem_1_binary */
    sem_1_binaryHandle = osSemaphoreNew(1, 0, &sem_1_binary_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */

    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* creation of timer_1 */
    //    timer_1Handle = osTimerNew(
    //            func_soft_timer, osTimerPeriodic, (void *)&t1, &timer_1_attributes);
    //
    //    /* creation of timer_2 */
    //    timer_2Handle = osTimerNew(
    //            func_soft_timer, osTimerOnce, (void *)&t2, &timer_2_attributes);
    //
    //    /* creation of timer_3 */
    //    timer_3Handle = osTimerNew(
    //            func_soft_timer, osTimerOnce, (void *)&t3, &timer_3_attributes);

    /* creation of timer_4 */
    //    timer_4Handle = osTimerNew(
    //            func_soft_timer, osTimerOnce, (void *)&t4, &timer_4_attributes);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    timer_1Handle = xTimerCreate(
            "timer_1", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_1_period,
            1,
            (void *)&t1,
            func_soft_timer);
    timer_2Handle = xTimerCreate(
            "timer_2", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_2_period,
            0,
            (void *)&t2,
            func_soft_timer);
    timer_3Handle = xTimerCreate(
            "timer_3", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_3_period,
            0,
            (void *)&t3,
            func_soft_timer);
    timer_4Handle = xTimerCreate(
            "timer_4", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
            timer_4_period,
            0,
            (void *)&t4,
            func_soft_timer);
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of queue1 */
    queue1Handle =
            osMessageQueueNew(5, sizeof(queue_element_t), &queue1_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle =
            osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of task_3 */
    task_3Handle = osThreadNew(
            func_task_from_queue, (void *)&par_3, &task_3_attributes);

    /* creation of task_4 */
    task_4Handle = osThreadNew(
            func_task_num_to_queue, (void *)&par_4, &task_4_attributes);

    /* creation of task_5 */
    task_5Handle = osThreadNew(
            func_task_txt_to_queue, (void *)&par_5, &task_5_attributes);

    /* creation of task_2 */
    task_2Handle = osThreadNew(
            func_button_handler, (void *)&par_2, &task_2_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    xTimerStart(timer_1Handle, 0);
    xTimerStart(timer_2Handle, 0);
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
        osDelay(1);
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
    osStatus_t      op_status = osOK;
    task_param_t   *ptr_str;
    queue_element_t q2;
    char            str_buf[50] = { 0 };

    ptr_str = (task_param_t *)argument;

    sprintf(str_buf, "%s \r\n", ptr_str->str);
    print(str_buf);
    /* Infinite loop */
    for (;;) {
        op_status = osMessageQueueGet(queue1Handle, &q2, NULL, portMAX_DELAY);
        if (op_status == osOK) {
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
    osStatus_t             op_status = osOK;
    task_param_t          *ptr_str;
    static queue_element_t q_el_num = { 0, TASK_SEND_NUMBER };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->led_period);
    /* Infinite loop */
    for (;;) {
        q_el_num.nmb++;
        op_status = osMessageQueuePut(queue1Handle, &q_el_num, 0, ticks_wait);
        if (op_status != osOK)
            print("Can`t send number to queue \r\n");

        osDelay(ticks_wait);
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
    osStatus_t             op_status = osOK;
    task_param_t          *ptr_str;
    static queue_element_t q_el_txt = { 0, TASK_SEND_TEXT };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->led_period);
    /* Infinite loop */
    for (;;) {
        if (q_el_txt.nmb < 9)
            q_el_txt.nmb++;
        else
            q_el_txt.nmb = 0;
        //op_status = xQueueSend(queue1Handle, &q_el_txt, ticks_wait);
        op_status = osMessageQueuePut(queue1Handle, &q_el_txt, 0, ticks_wait);
        if (op_status != osOK)
            print("Can`t send text to queue \r\n");

        osDelay(ticks_wait);
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
    osStatus_t             op_status = osOK;
    task_param_t          *ptr_str;
    static queue_element_t q_el_evt = { 11, TASK_SEND_TEXT };

    ptr_str = (task_param_t *)argument;

    const portTickType ticks_wait = pdMS_TO_TICKS(ptr_str->led_period);
    /* Infinite loop */
    while (1) {
        osSemaphoreAcquire(sem_1_binaryHandle, portMAX_DELAY);
        op_status = osMessageQueuePut(queue1Handle, &q_el_evt, 0, ticks_wait);
        if (op_status != osOK)
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
        xTimerStart(timer_3Handle, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    } else if (*timer_id == 3) {
        xTimerStart(timer_2Handle, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    } else if (*timer_id == 4) {
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
            osSemaphoreRelease(sem_1_binaryHandle);
        } else
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
        xTimerStartFromISR(timer_4Handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* USER CODE END Application */
