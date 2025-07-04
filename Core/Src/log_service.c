/*
 * log_service.c
 *
 *  Created on: Jul 1, 2025
 *      Author: andrii-vukolov
 */
#include "log_service.h"
#include "sensor_service.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define LOG_BUFFER_SIZE 100
#define LOG_MSG_SIZE    150

static char log_buffer[LOG_BUFFER_SIZE][LOG_MSG_SIZE] = { 0 };

TaskHandle_t  task_print;
TaskHandle_t  task_read_logs;
QueueHandle_t queue_print;

const char  empty[]   = { "" };
const char *txt_lvl[] = { "NONE ", "ERROR ", "WARNING ", "INFO ", "DEBUG " };

char *is_end   = log_buffer[LOG_BUFFER_SIZE];
char *is_start = log_buffer[0];
char *buf_head = log_buffer[0];
char *buf_tail = log_buffer[0];

void func_logger(log_lvl_t lvl, const char *log_msg, ...)
{
    if (lvl > LIBRARY_LOG_LEVEL)
        return;
    char     tmp_buf[LOG_MSG_SIZE] = { 0 };
    uint32_t time_stamp            = 0;
    time_stamp                     = (xTaskGetTickCount() / configTICK_RATE_HZ);

    va_list args;
    va_start(args, log_msg);
    sprintf(tmp_buf,
            "%s [%s] - %lu : %s\r\n",
            LIBRARY_LOG_NAME,
            txt_lvl[lvl],
            time_stamp,
            log_msg);
    vsprintf(buf_head, tmp_buf, args);
    va_end(args);

    buf_head += (LOG_MSG_SIZE * sizeof(char));
    if (buf_head >= is_end)
        buf_head = is_start;
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
        NOTE_ERROR("Can`t create read_logs task;");
    }
    return op_status;
}
