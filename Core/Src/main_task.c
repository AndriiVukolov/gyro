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

TaskHandle_t      task_main;
SemaphoreHandle_t data_put_block = NULL;

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

static servise_status_type_t angle_calc(queue_data_element_t *in_elt,
                                        queue_data_element_t *out_elt)
{
    out_elt->val_x = in_elt->val_x;
    out_elt->val_y = in_elt->val_y;
    out_elt->val_z = in_elt->val_z;
    return STATUS_OK;
}

static servise_status_type_t gui_data_transfer(void)
{
    servise_status_type_t stat;
    BaseType_t            op_status    = pdFAIL;
    gui_frame_data_t      elt_out      = { 0 };
    queue_data_element_t  accel_elt_in = { 0 };
    queue_data_element_t  gyro_elt_in  = { 0 };
    queue_data_element_t  angle_elt    = { 0 };
    //get data from gyro queue and place it to buffer gyro_elt_in
    stat = gyro_get(&gyro_elt_in);
    if (stat == STATUS_OK) {
        elt_out.pitch_vel = gyro_elt_in.val_x;
        elt_out.roll_vel  = gyro_elt_in.val_y;
        elt_out.yaw_vel   = gyro_elt_in.val_z;
        elt_out.raw_angle = gyro_elt_in.timestamp;
    }
    //get data from accel queue and place it to buffer accel_elt_in
    stat |= accel_get(&accel_elt_in);
    if (stat == STATUS_OK) {
        elt_out.pitch_acc = accel_elt_in.val_x;
        elt_out.roll_acc  = accel_elt_in.val_y;
        elt_out.yaw_acc   = accel_elt_in.val_z;
        elt_out.raw_angle = accel_elt_in.timestamp;
    }
    //calculate
    stat |= angle_calc(&accel_elt_in, &angle_elt);
    if (stat == STATUS_OK) {
        elt_out.pitch_ang = angle_elt.val_x;
        elt_out.roll_ang  = angle_elt.val_y;
        elt_out.yaw_ang   = angle_elt.val_z;
        //elt_out.raw_angle = angle_elt.timestamp;
    }
    //put frame with prepared data to GUI queue
    if (stat == STATUS_OK)
        op_status = gui_queue_data_put(&elt_out);
    if (op_status != pdFAIL)
        stat = STATUS_OK;
    return stat;
}

static void func_main(void *argument)
{
    servise_status_type_t stat = STATUS_OK;
    data_put_block             = xSemaphoreCreateMutex();

    while (1) {
        red_led_toggle();

        stat = gui_data_transfer();

        if (stat != STATUS_OK) {
            NOTE_ERROR("Can`t save GUI data to queue;");
        }
        vTaskDelay(TASK_TIMEOUT);
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
