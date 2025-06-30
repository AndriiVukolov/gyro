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
/*
 * creation for tasks -sensors_poll
 * creation for queues - gyro, accel, status
 * defines for queue size
 * functions for queue service (start/stop/get_data)
 * get_data func returns pointer to queue
 * */

#define QUEUE_SIZE 10

/**
 * Task parameters
 * */
typedef struct {
    void    *handler;
    uint32_t poll_period;
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

/**
 * @brief Creates and starts task for sensors poll and queues for measuring data and statuses
 * @param gyro_enable - if 1 - new task will be polling the Gyroscope, if 0 - Gyroscope data will be ignored
 * @param accel_enable -  if 1 - new task will be polling the Accelerometer, if 0 - Accelerometer data will be ignored
 * @returns status value
 * */
servise_status_type_t sensor_poll_start(uint8_t  gyro_enable,
                                        uint8_t  accel_enable,
                                        uint16_t queue_size);
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
servise_status_type_t gyro_queue_data_get(QueueHandle_t *data_queue);

/**
 * @brief Places the pointer to queue with Accelerometer data into *data_queue variable
 * @param  *data_queue
 * @returns status value
 * */
servise_status_type_t accel_queue_data_get(QueueHandle_t *data_queue);

/**
 * @brief Places the pointer to queue with statuses of sensors into *status_queue
 * @param  *status_queue
 * @returns status value
 * */
servise_status_type_t sensor_status_monitor(QueueHandle_t *status_queue);

#endif /* INC_SENSOR_SERVICE_H_ */
