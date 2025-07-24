/*
 * sensor_service.h
 *
 *  Created on: Jun 30, 2025
 *      Author: andrii-vukolov
 */

#ifndef INC_SENSOR_SERVICE_H_
#define INC_SENSOR_SERVICE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "gyro_I3G4250D.h"
#include "accel.h"
#include "timers.h"

#define QUEUE_SENSOR_DATA_SIZE 10
#define STATUS_POLL_PERIOD     pdMS_TO_TICKS(1000) //ms
#define SENSOR_POLL_PERIOD     pdMS_TO_TICKS(500)  //ms
#define PRINT_PERIOD           pdMS_TO_TICKS(1000) //ms
#define OPTIMAL_STACK_SIZE     1024

typedef enum { SENSOR_DISABLE, SENSOR_ENABLE } sensor_permission_t;
/**
 * Task parameters
 * */
typedef struct {
    sensor_permission_t gyro_perm;
    sensor_permission_t accel_perm;
} task_param_t;

/**
 * Element of queue with data
 * */
typedef struct {
    float    val_x;
    float    val_y;
    float    val_z;
    uint32_t timestamp;
} queue_data_element_t;

typedef enum {
    STATUS_OK,
    SENSOR_FAIL,
    COMMUNICATION_FAIL,
    SYSTEM_FAIL
} servise_status_type_t;
typedef enum { GYRO, ACCEL, SYSTEM } data_source_t;

typedef struct {
    servise_status_type_t status;
    data_source_t         source;
} stat_t;

/**
 * @brief Creates and starts task for sensors poll and queues for measuring data and statuses
 * @param gyro_enable - if 1 - new task will be polling the Gyroscope, if 0 - Gyroscope data will be ignored
 * @param accel_enable -  if 1 - new task will be polling the Accelerometer, if 0 - Accelerometer data will be ignored
 * @returns status value
 * */
servise_status_type_t sensor_poll_start(sensor_permission_t gyro_enable,
                                        sensor_permission_t accel_enable,
                                        uint16_t            queue_size);

/**
 * @brief Stop the task of sensor polling, clears memory
 * @returns status value
 * */
servise_status_type_t sensor_poll_stop(void);

/**
 * @brief Places the pointer to queue with Gyroscope data into *data_queue variable
 * @param  *data_queue
 * @returns status value
 * */
QueueHandle_t *gyro_queue_data_get(void);

/**
 * @brief Places the pointer to queue with Accelerometer data into *data_queue variable
 * @param  *data_queue
 * @returns status value
 * */
QueueHandle_t *accel_queue_data_get(void);

/**
 * @brief Places the pointer to queue with statuses of sensors into *status_queue
 * @param  *status_queue
 * @returns status value
 * */
QueueHandle_t *sensor_status_monitor(void);

#endif /* INC_SENSOR_SERVICE_H_ */
