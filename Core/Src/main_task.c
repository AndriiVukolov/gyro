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
#include "main_task.h"
#include "log_service.h"

static TaskHandle_t task_main;

static servise_status_type_t gui_gyro_data_put(void)
{
    gui_data_element_t   elt_out     = { 0 };
    queue_data_element_t elt_in      = { 0 };
    stat_t               task_status = { 0 };
    BaseType_t           op_status   = pdFAIL;
    QueueHandle_t       *qpointer    = NULL;
    qpointer                         = gyro_queue_data_get();
    op_status                        = xQueueReceive(*qpointer, &elt_in, 0);
    if (op_status != pdFAIL) {
        elt_out.val_x     = elt_in.val_x;
        elt_out.val_y     = elt_in.val_y;
        elt_out.val_z     = elt_in.val_z;
        elt_out.timestamp = elt_in.timestamp;
        elt_out.source    = GYRO;
        op_status         = gui_queue_data_put(&elt_out);
        NOTE_INFO("GYRO sent: %f, %f, %f ",
                  elt_out.val_x,
                  elt_out.val_y,
                  elt_out.val_z);
        task_status.status = STATUS_OK;
    } else {
        task_status.source = GYRO;
        task_status.status = SYSTEM_FAIL;
        NOTE_ERROR("Can't take GYRO data from GYRO_queue");
    }
    return task_status.status;
}
static servise_status_type_t gui_accel_data_put(void)
{
    gui_data_element_t   elt_out     = { 0 };
    queue_data_element_t elt_in      = { 0 };
    stat_t               task_status = { 0 };
    BaseType_t           op_status   = pdFAIL;
    QueueHandle_t       *qpointer    = NULL;
    qpointer                         = gyro_queue_data_get();
    op_status                        = xQueueReceive(*qpointer, &elt_in, 0);
    if (op_status != pdFAIL) {
        elt_out.val_x     = elt_in.val_x;
        elt_out.val_y     = elt_in.val_y;
        elt_out.val_z     = elt_in.val_z;
        elt_out.timestamp = elt_in.timestamp;
        elt_out.source    = ACCEL;
        op_status         = gui_queue_data_put(&elt_out);
        NOTE_INFO("ACCEL sent: %f, %f, %f ",
                  elt_out.val_x,
                  elt_out.val_y,
                  elt_out.val_z);
        task_status.status = STATUS_OK;
    } else {
        task_status.source = ACCEL;
        task_status.status = SYSTEM_FAIL;
        NOTE_ERROR("Can't take ACCEL data from ACCEL_queue");
    }
    return task_status.status;
}
static void func_main(void *argument)
{
    servise_status_type_t stat = SYSTEM_FAIL;

    while (1) {
        red_led_toggle();
        stat = gui_gyro_data_put();
        stat |= gui_accel_data_put();
        if (stat != STATUS_OK) {
            NOTE_ERROR("Can`t save GUI data to queue;");
        }
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
                            osPriorityBelowNormal1,
                            &task_main);
    if (op_status != pdPASS) {
        NOTE_ERROR("Can`t create main task;");
    }
    return op_status;
}
