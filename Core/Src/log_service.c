/*
 * log_service.c
 *
 *  Created on: Jul 1, 2025
 *      Author: andrii-vukolov
 */
#include "log_service.h"
#include "sensor_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "gyro_I3G4250D.h"
#include "accel.h"
#include "timers.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define LOG_BUFFER_SIZE 100
#define LOG_MSG_SIZE    150

static char log_buffer[LOG_BUFFER_SIZE][LOG_MSG_SIZE] = { 0 };

TaskHandle_t  task_print;
TaskHandle_t  task_read_logs;
QueueHandle_t queue_print;

char empty[] = { "" };

char *is_end   = log_buffer[LOG_BUFFER_SIZE];
char *is_start = log_buffer[0];
char *buf_head = log_buffer[0];
char *buf_tail = log_buffer[0];

void func_logger(char *str_to_buf)
{
    uint32_t time_stamp = 0;
    time_stamp          = (xTaskGetTickCount() / configTICK_RATE_HZ);

    sprintf(buf_head, "%lu : %s \r\n", time_stamp, str_to_buf);

    buf_head += (LOG_MSG_SIZE * sizeof(char));
    if (buf_head >= is_end)
        buf_head = is_start;
}

void func_logger_data(queue_data_element_t *elt)
{
    char *str_to_log = calloc(LOG_MSG_SIZE, sizeof(char));

    sprintf(str_to_log,
            "X=%f, Y=%f, Z=%f, Timestamp=%lu",
            elt->val_x,
            elt->val_y,
            elt->val_z,
            elt->timestamp);
    func_logger(str_to_log);
    free(str_to_log);
}

static void func_read_logs(void *arguments)
{
    red_led_toggle();

    while (1) {
        if (*buf_tail != *empty) {
            print(buf_tail);
            strcpy(buf_tail, empty);
            buf_tail += (LOG_MSG_SIZE * sizeof(char));
            if (buf_tail >= is_end)
                buf_tail = is_start;
        }
    }
}

void print(const char *str)
{
    if (HAL_UART_Transmit(&huart1, (const uint8_t *)str, strlen(str), 1000) !=
        HAL_OK) {
        Error_Handler();
    }
}

BaseType_t read_log_service_start(void)
{
    BaseType_t op_status = pdFAIL;

    op_status = xTaskCreate(func_read_logs,
                            "task_read_logs",
                            (OPTIMAL_STACK_SIZE),
                            NULL,
                            (osPriorityLow1),
                            &task_read_logs);

    if (op_status == pdFAIL) {
        func_logger("Can`t create read_logs task; \r\n");
    }
    return op_status;
}
