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
task_param_t par_1 = { NULL, 3, 1000 };
task_param_t par_2 = { NULL, 4, 2500 };
task_param_t par_3 = { " Hello from FreeRTOS ! ", 0, 3000 };
task_param_t par_4 = { NULL, 0, 1000 };
task_param_t par_5 = { NULL, 0, 2000 };

uint32_t t1             = 1;
uint32_t t2             = 2;
uint32_t t3             = 3;
uint32_t timer_1_period = pdMS_TO_TICKS(1000);
uint32_t timer_2_period = pdMS_TO_TICKS(300);
uint32_t timer_3_period = pdMS_TO_TICKS(2000);

const char digits_txt[10][10] = { { "ZERO" },  { "ONE" },   { "TWO" },
                                  { "THREE" }, { "FOUR" },  { "FIVE" },
                                  { "SIX" },   { "SEVEN" }, { "EIGHT" },
                                  { "NINE" } };
/* USER CODE END Variables */
/* Definitions for defaultTask */

osThreadId_t         defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for task_3 */
osThreadId_t         task_3Handle;
const osThreadAttr_t task_3_attributes = {
    .name       = "task_3",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow2,
};
/* Definitions for task_4 */
osThreadId_t         task_4Handle;
const osThreadAttr_t task_4_attributes = {
    .name       = "task_4",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow1,
};
/* Definitions for task_5 */
osThreadId_t         task_5Handle;
const osThreadAttr_t task_5_attributes = {
    .name       = "task_5",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow1,
};
/* Definitions for queue1 */
osMessageQueueId_t         queue_1;
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
TimerHandle_t timer_1;
TimerHandle_t timer_2;
TimerHandle_t timer_3;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void func_task_from_queue(void *argument);
void func_task_num_to_queue(void *argument);
void func_task_txt_to_queue(void *argument);
void func_timer(void *argument);
void func_soft_timer(TimerHandle_t xTimer);

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

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* creation of timer_1 */
    //    timer_1Handle = osTimerNew(
    //            func_soft_timer, osTimerPeriodic, (void *)&t1, &timer_1_attributes);

    timer_1 = xTimerCreate(
            "timer_1", timer_1_period, 1, (void *)&t1, func_soft_timer);

    /* creation of timer_2 */
    //    timer_2 = osTimerNew(
    //            func_soft_timer, osTimerOnce, (void *)&t2, &timer_2_attributes);

    timer_2 = xTimerCreate(
            "timer_2", timer_2_period, 0, (void *)&t2, func_soft_timer);

    /* creation of timer_3 */
    //    timer_3 = osTimerNew(
    //            func_soft_timer, osTimerOnce, (void *)&t3, &timer_3_attributes);

    timer_3 = xTimerCreate(
            "timer_3", timer_3_period, 0, (void *)&t3, func_soft_timer);
    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of queue1 */
    queue_1 = osMessageQueueNew(5, sizeof(queue_element_t), &queue1_attributes);

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

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    xTimerStart(timer_1, 0);
    xTimerStart(timer_2, 0);
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
        op_status = osMessageQueueGet(queue_1, &q2, NULL, portMAX_DELAY);
        if (op_status == osOK) {
            if (q2.src == TASK_SEND_NUMBER)
                sprintf(str_buf, "%lu \r\n", q2.nmb);
            else {
                sprintf(str_buf, "%s \r\n", digits_txt[q2.nmb]);
            }
            print(str_buf);
        } else
            print("The queue is empty! \r\n");
        //osDelay(1);
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
        op_status = osMessageQueuePut(queue_1, &q_el_num, 0, ticks_wait);
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
        op_status = osMessageQueuePut(queue_1, &q_el_txt, 0, ticks_wait);
        if (op_status != osOK)
            print("Can`t send text to queue \r\n");

        osDelay(ticks_wait);
    }
    /* USER CODE END func_task_txt_to_queue */
}

/* func_timer function */

/* func_soft_timer function */
void func_soft_timer(TimerHandle_t xTimer)
{
    /* USER CODE BEGIN func_soft_timer */
    //uint32_t timer_number = *(uint32_t *)argument;
    //    osTimerId_t timer_id;
    uint32_t *timer_id = NULL;

    timer_id = pvTimerGetTimerID(xTimer);

    if (*timer_id == 1) {
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    } else if (*timer_id == 2) {
        xTimerStart(timer_3, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

    } else if (*timer_id == 3) {
        xTimerStart(timer_2, 0);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    }

    /* USER CODE END func_soft_timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
