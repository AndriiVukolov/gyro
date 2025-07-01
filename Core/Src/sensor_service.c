/*
 * sensor_service.c
 *
 *  Created on: Jun 30, 2025
 *      Author: andrii-vukolov
 */
#include "sensor_service.h"
#include "spi.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

TaskHandle_t task_sensor_handler;
TaskHandle_t task_print;

QueueHandle_t queue_print;
QueueHandle_t queue_gyro, queue_accel, queue_status;

gyro_t       g1               = { 0 };
accel_t      a1               = { 0 };
task_param_t sensor_task_data = { 0 };

/**
 * @brief function transmits text over uart
 * @param *str - pointer to string for transmission
 *
 * */
void print(const char *str)
{
    if (HAL_UART_Transmit(&huart1, (const uint8_t *)str, strlen(str), 10) !=
        HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  Reads measured data from Gyroscope over SPI
 * */
static gyroError_t
func_read_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = GYRO_OK;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    if (len > 1) {
        txByte = (READ << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (READ << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }

    GYRO_SPI_EN();
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);

    for (uint8_t x = 0; x < len; x++) {
        txByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
        store[x] = rxByte;
        rxByte   = 0;
    }
    GYRO_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;

    return gerr;
}

static gyroError_t
func_write_gyro_spi(uint8_t startAdd, uint16_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = GYRO_OK;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    GYRO_SPI_EN();
    if (len > 1) {
        txByte = (WRITE << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (WRITE << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    for (uint8_t x = 0; x < len; x++) {
        txByte = store[x];
        rxByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    }
    GYRO_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;
    return gerr;
}

static accel_error_t
func_read_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = GYRO_OK;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    if (len > 1) {
        txByte = (READ << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (READ << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }

    ACCEL_SPI_EN();
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);

    for (uint8_t x = 0; x < len; x++) {
        txByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
        store[x] = rxByte;
        rxByte   = 0;
    }
    ACCEL_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;

    return gerr;
}

static accel_error_t
func_write_accel_spi(uint8_t startAdd, uint8_t len, uint8_t *store)
{
    HAL_StatusTypeDef err    = HAL_OK;
    gyroError_t       gerr   = GYRO_OK;
    uint8_t           rxByte = 0;
    uint8_t           txByte = 0;

    ACCEL_SPI_EN();
    if (len > 1) {
        txByte = (WRITE << 7) | (MULTIPLY << 6) | (startAdd & 0x3F);
    } else {
        txByte = (WRITE << 7) | (SINGLE << 6) | (startAdd & 0x3F);
    }
    err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    for (uint8_t x = 0; x < len; x++) {
        txByte = store[x];
        rxByte = 0;
        err = HAL_SPI_TransmitReceive(&hspi5, &txByte, &rxByte, 1, SPI_TIMEOUT);
    }
    ACCEL_SPI_DIS();
    if (err != HAL_OK)
        gerr = gyroCommError;
    return gerr;
}

static void func_sensors_poll(void *argument)
{
    gyroError_t          gerr        = GYRO_OK;
    accel_error_t        aerr        = ACCEL_OK;
    float                vals[3]     = { 0 };
    queue_data_element_t sensor_data = { 0 };
    queue_data_element_t dummy       = { 0 };
    BaseType_t           op_status   = pdFAIL;

    stat_t task_status = { 0 };

    while (1) {
        green_led_toggle();
        if (sensor_task_data.gyro_perm == SENSOR_ENABLE) {
            gerr = gyroReadVal(&g1, vals);
            if (gerr == GYRO_OK) {
                sensor_data.val_x     = vals[0];
                sensor_data.val_y     = vals[1];
                sensor_data.val_z     = vals[2];
                sensor_data.timestamp = xTaskGetTickCount(); //ms

                if ((sensor_data.val_x == 0) && (sensor_data.val_y == 0) &&
                    (sensor_data.val_z == 0)) {
                    task_status.status = SENSOR_FAIL;
                    task_status.source = GYRO;
                }

                op_status = xQueueSend(
                        queue_gyro, &sensor_data, SENSOR_POLL_PERIOD);
                if (op_status != pdPASS) {
                    op_status &= xQueueReceive(queue_gyro, &dummy, 0);
                    op_status &= xQueueSend(
                            queue_gyro, &sensor_data, SENSOR_POLL_PERIOD);
                    if (op_status != pdPASS) {
                        task_status.status = SYSTEM_FAIL;
                        task_status.source = SYSTEM;
                    }
                }
            } else {
                task_status.status = SENSOR_FAIL;
                task_status.source = GYRO;
            }
        }
        if (sensor_task_data.accel_perm == SENSOR_ENABLE) {
            aerr = accel_data_get(&a1, vals);
            if (aerr == ACCEL_OK) {
                sensor_data.val_x     = vals[0];
                sensor_data.val_y     = vals[1];
                sensor_data.val_z     = vals[2];
                sensor_data.timestamp = xTaskGetTickCount(); //ms

                if ((sensor_data.val_x == 0) && (sensor_data.val_y == 0) &&
                    (sensor_data.val_z == 0)) {
                    task_status.status = SENSOR_FAIL;
                    task_status.source = ACCEL;
                }

                op_status &= xQueueSend(
                        queue_accel, &sensor_data, SENSOR_POLL_PERIOD);
                if (op_status != pdPASS) {
                    op_status &= xQueueReceive(queue_accel, &dummy, 0);
                    op_status &= xQueueSend(
                            queue_accel, &sensor_data, SENSOR_POLL_PERIOD);
                    if (op_status != pdPASS) {
                        task_status.status = SYSTEM_FAIL;
                        task_status.source = SYSTEM;
                    }
                }
            } else {
                task_status.status = SENSOR_FAIL;
                task_status.source = ACCEL;
            }
        }

        op_status = xQueueSend(queue_status, &task_status, STATUS_POLL_PERIOD);
        if (op_status != pdPASS) {
            op_status &= xQueueReceive(queue_accel, &dummy, 0);
            op_status &=
                    xQueueSend(queue_accel, &task_status, SENSOR_POLL_PERIOD);
            if (op_status != pdPASS) {
                task_status.status = SYSTEM_FAIL;
                task_status.source = SYSTEM;
            }
        }
        vTaskDelay(SENSOR_POLL_PERIOD);
    }
}

static void func_print(void *argument)
{

    char                 print_element[100] = { 0 };
    queue_data_element_t gyro_element       = { 0 };
    queue_data_element_t accel_element      = { 0 };
    stat_t               event_element      = { 0 };
    BaseType_t           data_status        = pdFAIL;

    while (1) {
        red_led_toggle();
        if (sensor_task_data.gyro_perm == SENSOR_ENABLE) {
            data_status = xQueueReceive(
                    *(gyro_queue_data_get()), &gyro_element, PRINT_PERIOD);
            if (data_status != pdFAIL) {
                sprintf(print_element,
                        "GYRO: X=%f, Y=%f, Z=%f, Timestamp=%lu \r\n",
                        gyro_element.val_x,
                        gyro_element.val_y,
                        gyro_element.val_z,
                        gyro_element.timestamp);
            } else
                sprintf(print_element, "There is no Gyroscope data \r\n");

            print(print_element);
        }
        if (sensor_task_data.accel_perm == SENSOR_ENABLE) {
            data_status = xQueueReceive(
                    *(accel_queue_data_get()), &accel_element, PRINT_PERIOD);
            if (data_status != pdFAIL) {
                sprintf(print_element,
                        "ACCEL: X=%f, Y=%f, Z=%f, Timestamp=%lu \r\n",
                        accel_element.val_x,
                        accel_element.val_y,
                        accel_element.val_z,
                        accel_element.timestamp);
            } else
                sprintf(print_element, "There is no Accelerometer data \r\n");
            print(print_element);
        }
        data_status = xQueueReceive(
                *(sensor_status_monitor()), &event_element, PRINT_PERIOD);
        if ((data_status != pdFAIL) && (event_element.status != STATUS_OK)) {
            sprintf(print_element,
                    "Error code %u%u \r\n",
                    event_element.source,
                    event_element.status);
            print(print_element);
        }
    }
    vTaskDelay(PRINT_PERIOD);
}

servise_status_type_t sensor_poll_start(sensor_permission_t gyro_enable,
                                        sensor_permission_t accel_enable,
                                        uint16_t            queue_size)
{
    gyroError_t   gerr      = GYRO_OK;
    accel_error_t aerr      = ACCEL_OK;
    BaseType_t    op_status = pdFAIL;
    //===============================================================================QUEUES creation
    queue_gyro = xQueueCreate(queue_size, sizeof(queue_data_element_t));
    if (gyro_enable == SENSOR_ENABLE) {
        //===========================================================================GYRO Init
        sensor_task_data.gyro_perm = SENSOR_ENABLE;
        g1.data_read               = func_read_gyro_spi;
        g1.data_write              = func_write_gyro_spi;
        gerr                       = gyroInit(&g1);
        if (gerr != GYRO_OK)
            return (SYSTEM_FAIL);
        //===========================================================================END OF GYRO INIT
        if (queue_gyro == NULL) {
            return (SYSTEM_FAIL);
        }
    } else
        sensor_task_data.gyro_perm = SENSOR_DISABLE;

    queue_accel = xQueueCreate(queue_size, sizeof(queue_data_element_t));
    if (accel_enable == SENSOR_ENABLE) {
        //==========================================================================ACCEL INIT
        sensor_task_data.accel_perm = SENSOR_ENABLE;
        a1.data_write               = func_write_accel_spi;
        a1.data_read                = func_read_accel_spi;
        aerr                        = accel_init(&a1); // set defines
        if (aerr != ACCEL_OK)
            return (SYSTEM_FAIL);
        //=========================================================================END OF ACCEL INIT
        if (queue_accel == NULL) {
            return (SYSTEM_FAIL);
        }
    } else
        sensor_task_data.accel_perm = SENSOR_DISABLE;

    queue_status = xQueueCreate(queue_size, sizeof(stat_t));
    if (queue_status == NULL) {
        return (SYSTEM_FAIL);
    }
    queue_print = xQueueCreate(4, sizeof(char[100]));
    if (queue_print == NULL) {
        return (SYSTEM_FAIL);
    }
    //===========================================================================TASK creation
    op_status = xTaskCreate(func_sensors_poll,
                            "task_sensors_handler",
                            (OPTIMAL_STACK_SIZE),
                            (void *)&sensor_task_data,
                            osPriorityBelowNormal1,
                            &task_sensor_handler);
    op_status &= xTaskCreate(func_print,
                             "task_print",
                             OPTIMAL_STACK_SIZE,
                             (void *)&sensor_task_data,
                             osPriorityBelowNormal1,
                             &task_print);
    if (op_status != pdPASS) {
        return (SYSTEM_FAIL);
    }
    return (STATUS_OK);
}

servise_status_type_t sensor_poll_stop(void)
{

    vTaskDelete(task_sensor_handler);
    vTaskDelete(task_print);
    vQueueDelete(queue_gyro);
    vQueueDelete(queue_accel);
    vQueueDelete(queue_status);

    return (STATUS_OK);
}

QueueHandle_t *gyro_queue_data_get(void)
{
    return (&queue_gyro);
}

QueueHandle_t *accel_queue_data_get(void)
{

    return (&queue_accel);
}

QueueHandle_t *sensor_status_monitor(void)
{

    return (&queue_status);
}
