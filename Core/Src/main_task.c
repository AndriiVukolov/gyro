/**
 * @file main_task.c
 * @brief Main module
 * This module is for maintenance data transfer from sensors service to GUI service
 *
 *  Created on: Jul 24, 2025
 *  @author: Andrii V.
 *  @copyright (C) 2025 Alnicko Development OU. All rights reserved.
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "main_task.h"
#include "log_service.h"
#include "math.h"

TaskHandle_t      task_main;
SemaphoreHandle_t data_put_block = NULL;
float             cline_angle    = 0;
uint8_t           reset_flag     = 0;

/**
 * @brief Function pulls data from GYRO queue, and pushes it to GUI queue with additional "source" field
 * @return STATUS_OK = 0, or SYSTEM_FAIL = 1;
 * */
static servise_status_type_t accel_get(queue_data_element_t *aelt)
{
    servise_status_type_t res       = SYSTEM_FAIL;
    BaseType_t            op_status = pdFAIL;
    QueueHandle_t        *qptr;
    qptr = accel_queue_data_get();

    op_status = xQueueReceive(*qptr, aelt, QUEUE_RECEIVE_TIMEOUT);
    if (op_status == pdPASS)
        res = STATUS_OK;

    return res;
}

static servise_status_type_t gyro_get(queue_data_element_t *gelt)
{
    BaseType_t            op_status = pdFAIL;
    servise_status_type_t res       = SYSTEM_FAIL;
    QueueHandle_t        *qptr;
    qptr = gyro_queue_data_get();

    op_status = xQueueReceive(*qptr, gelt, QUEUE_RECEIVE_TIMEOUT);
    if (op_status == pdPASS)
        res = STATUS_OK;

    return res;
}

static servise_status_type_t tilt_angle_calc(queue_data_element_t *in_elt,
                                             queue_data_element_t *out_elt)
{

    if ((in_elt->val_x == 0) && (in_elt->val_y == 0) && (in_elt->val_z == 0))
        return SENSOR_FAIL;
    else {
        float x        = in_elt->val_x;
        float y        = in_elt->val_y;
        float z        = in_elt->val_z;
        out_elt->val_x = (180 / M_PI) * acos(x / sqrt(z * z + x * x));
        out_elt->val_y = (180 / M_PI) * acos(y / sqrt(x * x + y * y));
        out_elt->val_z = (180 / M_PI) * acos(z / sqrt(z * z + y * y));
    }
    return STATUS_OK;
}
static float yaw_angle_calc(queue_data_element_t *gyro_elt)
{
    static float angle = 0;
    float        v     = gyro_elt->val_z;

    if (reset_flag == 1) {
        angle      = 0;
        reset_flag = 0;
    } else {
        if ((v > GYRO_NORMAL_ERROR) || (v < (-GYRO_NORMAL_ERROR)))
            angle -= ((v * TASK_TIMEOUT) / configTICK_RATE_HZ);
    }
    return angle;
}

static servise_status_type_t
gui_data_transfer(queue_data_element_t *gyro_elt_in,
                  queue_data_element_t *accel_elt_in,
                  queue_data_element_t *angle_elt)

{
    servise_status_type_t stat;
    BaseType_t            op_status = pdFAIL;
    gui_frame_data_t      elt_out   = { 0 };

    //get data from gyro queue and place it to buffer gyro_elt_in

    elt_out.pitch_vel  = gyro_elt_in->val_x;
    elt_out.roll_vel   = gyro_elt_in->val_y;
    elt_out.yaw_vel    = gyro_elt_in->val_z;
    elt_out.line_angle = (float)gyro_elt_in->timestamp;

    //get data from accel queue and place it to buffer accel_elt_in

    elt_out.pitch_acc  = accel_elt_in->val_x / 100;
    elt_out.roll_acc   = accel_elt_in->val_y / 100;
    elt_out.yaw_acc    = accel_elt_in->val_z / 100;
    elt_out.line_angle = (float)accel_elt_in->timestamp;

    //calculate

    elt_out.pitch_ang = angle_elt->val_x;
    elt_out.roll_ang  = angle_elt->val_y;
    elt_out.yaw_ang   = angle_elt->val_z;

    //put frame with prepared data to GUI queue

    op_status = gui_queue_data_put(&elt_out);
    if (op_status != pdFAIL)
        stat = STATUS_OK;
    return stat;
}

void main_reset_flag_set(void)
{
    reset_flag = 1;
}

static void func_main(void *argument)
{
    servise_status_type_t stat           = STATUS_OK;
    queue_data_element_t  gyro_elt       = { 0 };
    queue_data_element_t  accel_elt      = { 0 };
    queue_data_element_t  angle_elt      = { 0 };
    TickType_t            last_wake_time = xTaskGetTickCount();

    while (1) {
        red_led_toggle();
        stat = gyro_get(&gyro_elt);
        stat |= accel_get(&accel_elt);
        stat |= tilt_angle_calc(&accel_elt, &angle_elt);
        angle_elt.val_z = yaw_angle_calc(&gyro_elt);
        stat |= gui_data_transfer(&gyro_elt, &accel_elt, &angle_elt);

        if (stat != STATUS_OK) {
            NOTE_ERROR("Can`t save GUI data to queue;");
        }

        vTaskDelayUntil(&last_wake_time, TASK_TIMEOUT);
    }
}
BaseType_t func_main_start(void)
{
    BaseType_t op_status = pdFAIL;

    //============================================================================TASK creation
    op_status = xTaskCreate(func_main,
                            "task_gui_frame_draw",
                            (OPTIMAL_STACK_SIZE),
                            NULL,
                            osPriorityLow7,
                            &task_main);
    if (op_status != pdPASS) {
        NOTE_ERROR("Can`t create main task;");
    }
    return op_status;
}
