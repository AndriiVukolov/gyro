/*
 * sensor_service.c
 *
 *  Created on: Jun 30, 2025
 *      Author: andrii-vukolov
 */
#include "sensor_service.h"
#include "spi.h"
#include "log_service.h"
#include "gui.h"

static vector_t vector_raw = { 0 };

static TaskHandle_t task_sensor_handler;
static TaskHandle_t task_read_logs;

static QueueHandle_t queue_gyro, queue_accel, queue_status;

static gyro_t       g1               = { 0 };
static accel_t      a1               = { 0 };
static task_param_t sensor_task_data = { 0 };

static uint32_t an = 0;

/**
 * @brief function transmits text over uart
 * @param *str - pointer to string for transmission
 *
 * */

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
    stat_t               task_status = { 0 };
    TickType_t           last_wake_time;
    last_wake_time = xTaskGetTickCount();

    while (1) {
        green_led_toggle();
        if (sensor_task_data.gyro_perm == SENSOR_ENABLE) {
            //================================================================Read GYRO data
            gerr = gyroReadVal(&g1, vals);
            if (gerr == GYRO_OK) {
                sensor_data.val_x = vals[0];
                sensor_data.val_y = vals[1];
                sensor_data.val_z = vals[2];
                sensor_data.timestamp =
                        (xTaskGetTickCount() * 1000 / configTICK_RATE_HZ); //ms

                //==============================================================Test log with received values
                NOTE_INFO("GYRO received: %f, %f, %f ",
                          sensor_data.val_x,
                          sensor_data.val_y,
                          sensor_data.val_z);

                //==============================================================Check if the data is valid
                if ((sensor_data.val_x == 0) && (sensor_data.val_y == 0) &&
                    (sensor_data.val_z == 0) && (sensor_data.timestamp == 0)) {
                    NOTE_WARN(
                            "Measured values of Gyroscope are all equal to zero - it is may be fail;");
                } else
                    NOTE_DEBUG("GYROSCOPE data successful received;");
                //===============================================================Try to place the data to queue
                op_status = xQueueSend(queue_gyro, &sensor_data, 0);
                if (op_status != pdPASS) {
                    op_status = xQueueReceive(queue_gyro, &dummy, 0);
                    op_status &= xQueueSend(queue_gyro, &sensor_data, 0);
                    if (op_status != pdPASS) {
                        task_status.status = SYSTEM_FAIL;
                        task_status.source = SYSTEM;
                        NOTE_ERROR("Can`t save Gyroscope data due to queue;");
                    }
                }
            } else {
                task_status.status = SENSOR_FAIL;
                task_status.source = GYRO;
                NOTE_ERROR("Can`t read Gyroscope data;");
            }
        }
        if (sensor_task_data.accel_perm == SENSOR_ENABLE) {
            //================================================================Read ACCEL data
            aerr = accel_data_get(&a1, vals);
            if (aerr == ACCEL_OK) {
                sensor_data.val_x = vals[0];
                sensor_data.val_y = vals[1];
                sensor_data.val_z = vals[2];
                sensor_data.timestamp =
                        (xTaskGetTickCount() * 1000 / configTICK_RATE_HZ); //ms
                //==============================================================Test log with received values
                NOTE_INFO("ACCEL received: %f, %f, %f ",
                          sensor_data.val_x,
                          sensor_data.val_y,
                          sensor_data.val_z);
                //==============================================================Check if the data is valid
                if ((sensor_data.val_x == 0) && (sensor_data.val_y == 0) &&
                    (sensor_data.val_z == 0) && (sensor_data.timestamp == 0)) {
                    NOTE_WARN(
                            "Measured values of Accelerometer are all equal to zero - it is maybe Antigravity!;");
                } else
                    NOTE_DEBUG("ACCELEROMETER data successful received;");
                //===============================================================Try to place the data to queue
                op_status &= xQueueSend(queue_accel, &sensor_data, 0);
                if (op_status != pdPASS) {
                    op_status = xQueueReceive(queue_accel, &dummy, 0);
                    op_status &= xQueueSend(queue_accel, &sensor_data, 0);
                    if (op_status != pdPASS) {
                        task_status.status = SYSTEM_FAIL;
                        task_status.source = SYSTEM;
                        NOTE_ERROR("Can`t save Accelerometer data;");
                    }
                }
            } else {
                task_status.status = SENSOR_FAIL;
                task_status.source = ACCEL;
                NOTE_ERROR("Can`t read Accelerometer data;");
            }
        }


        op_status = xQueueSend(queue_status, &task_status, 0);
        if (op_status != pdPASS) {
            op_status = xQueueReceive(queue_accel, &dummy, 0);
            op_status &= xQueueSend(queue_accel, &task_status, 0);
            if (op_status != pdPASS) {
                task_status.status = SYSTEM_FAIL;
                task_status.source = SYSTEM;
                NOTE_ERROR("Can`t save status data;");
            }
            NOTE_DEBUG("STATUS data successful saved;");
        } else
            NOTE_DEBUG("STATUS data successful saved;");
        vTaskDelayUntil(&last_wake_time, SENSOR_POLL_PERIOD);
    }
}

servise_status_type_t sensor_poll_start(sensor_permission_t gyro_enable,
                                        sensor_permission_t accel_enable,
                                        uint16_t            queue_size)
{
    gyroError_t   gerr      = GYRO_OK;
    accel_error_t aerr      = ACCEL_OK;
    BaseType_t    op_status = pdFAIL;
    //===============================================================================QUEUES creation

    if (gyro_enable == SENSOR_ENABLE) {
        //===========================================================================GYRO Init
        sensor_task_data.gyro_perm = SENSOR_ENABLE;
        g1.data_read               = func_read_gyro_spi;
        g1.data_write              = func_write_gyro_spi;
        gerr                       = gyroInit(&g1);
        if (gerr != GYRO_OK) {
            NOTE_ERROR("Can`t initialize the Gyroscope;");
            return (SYSTEM_FAIL);
        }

        queue_gyro = xQueueCreate(queue_size, sizeof(queue_data_element_t));
        //===========================================================================END OF GYRO INIT
        if (queue_gyro == NULL) {
            NOTE_ERROR("Can`t create the queue for the Gyroscope data;");
            return (SYSTEM_FAIL);
        }
        NOTE_INFO("The Gyroscope is initialized and running;");
    } else {
        sensor_task_data.gyro_perm = SENSOR_DISABLE;
        queue_gyro                 = NULL;
        NOTE_INFO("The Gyroscope is disabled;");
    }

    if (accel_enable == SENSOR_ENABLE) {
        //==========================================================================ACCEL INIT
        sensor_task_data.accel_perm = SENSOR_ENABLE;
        a1.data_write               = func_write_accel_spi;
        a1.data_read                = func_read_accel_spi;
        aerr                        = accel_init(&a1); // set defines
        if (aerr != ACCEL_OK) {
            NOTE_ERROR("Can`t initialize the Accelerometer;");
            return (SYSTEM_FAIL);
        }

        queue_accel = xQueueCreate(queue_size, sizeof(queue_data_element_t));
        //=========================================================================END OF ACCEL INIT
        if (queue_accel == NULL) {
            NOTE_ERROR("Can`t create the queue for the Accelerometer data;");
            return (SYSTEM_FAIL);
        }
        NOTE_INFO("The Accelerometer is initialized and running;");
    } else {
        sensor_task_data.accel_perm = SENSOR_DISABLE;
        queue_accel                 = NULL;
        NOTE_INFO("The Accelerometer is disabled;");
    }

    queue_status = xQueueCreate(queue_size, sizeof(stat_t));
    if (queue_status == NULL) {
        NOTE_ERROR("Can`t create the queue for the Status data;");
        return (SYSTEM_FAIL);
    }

    //===========================================================================TASK creation
    op_status = xTaskCreate(func_sensors_poll,
                            "task_sensors_handler",
                            (OPTIMAL_STACK_SIZE),
                            (void *)&sensor_task_data,
                            osPriorityBelowNormal1,
                            &task_sensor_handler);

    if (op_status != pdPASS) {
        return (SYSTEM_FAIL);
    }

    NOTE_INFO("The System is initialized and running;");

    return (STATUS_OK);
}

servise_status_type_t sensor_poll_stop(void)
{

    vTaskDelete(task_sensor_handler);
    vTaskDelete(task_read_logs);
    vQueueDelete(queue_gyro);
    vQueueDelete(queue_accel);
    vQueueDelete(queue_status);
    NOTE_INFO("The Sensor_service is stopped, data memory is cleared;");

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
