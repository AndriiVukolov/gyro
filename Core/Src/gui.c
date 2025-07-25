/*
 * gui.c
 *
 *  Created on: Jul 7, 2025
 *      Author: andrii-vukolov
 */
#include "gui.h"
#include "ili9341.h"

#include "lcd.h"
#include <math.h>
#include "main.h"
#include <stdio.h>
#include "log_service.h"

static TaskHandle_t  task_gui_frame;
static QueueHandle_t queue_gui; //queue of data to be displayed
//static gui_frame_data_t gui_frame_data = { 0 };

typedef struct {
    uint32_t coord_x;
    uint32_t coord_y;
    uint32_t color;
} pixel_t;

typedef struct {
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    uint32_t len;
    uint32_t color;
} line_t;

typedef struct {
    uint32_t s_x; //Coordinate x of begin
    uint32_t s_y; //Coordinate y of begin
    uint32_t lng; //length of vector
    uint32_t deg; //angle between north direction and vector direction (0 - 359 deg)
} vector_t;

static line_t ln = { 0 }; //

/**
 * @brief Set params of line to be drawn
 * */
static void gui_set_compass_line(uint32_t color, uint32_t len)
{
    ln.x1    = BSP_LCD_GetXSize() / 2;
    ln.y1    = BSP_LCD_GetYSize() / 2 + BSP_LCD_GetYSize() / 4;
    ln.len   = len;
    ln.x2    = ln.x1 + ln.len;
    ln.y2    = ln.y1;
    ln.color = color;
}

/**
 * @brief Draws line at the center of bottom of lcd turned in
 * @param degrees - angle degrees between the line the nord direction
 * */
static void gui_draw_compass_line(double degrees)
{
    double angle_radians = degrees * M_PI / 180.0;

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(ln.x1, ln.y1, ln.x2, ln.y2);
    ln.x2 = ln.x1 + (ln.len * sin(angle_radians));
    ln.y2 = ln.y1 + (ln.len * cos(angle_radians));
    BSP_LCD_SetTextColor(ln.color);
    BSP_LCD_DrawLine(ln.x1, ln.y1, ln.x2, ln.y2);
}

/**
 * @brief Draws text string with angle value of compass line direction
 * @param degrees - angle value to draw
 * */
static void gui_draw_pitch_val(double degrees)
{
    uint8_t str_buf[MAX_TXT_LINE_LEN] = { 0 };

    sprintf((char *)str_buf, "YAW ANGLE: %3.2f", degrees);
    BSP_LCD_ClearStringLine(Y_POS_STRING_MAIN);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
    BSP_LCD_DisplayStringAt(
            X_POS_STRING, Y_POS_STRING_MAIN, str_buf, LEFT_MODE);
}

/**
 * @brief Clears whole display (fills it by background color) *
 * */
static void gui_clear(void)
{
    BSP_LCD_Clear(LCD_COLOR_WHITE);
}

/**
 * @brief Draws horizontal line in the middle of display
 * */
static void gui_draw_border(void)
{
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
    BSP_LCD_DrawHLine(X_BORDER_POS, Y_BORDER_POS, BSP_LCD_GetXSize());
}

static void gui_draw_pry_val(float yaw_ang,
                             float pitch_ang,
                             float roll_ang,
                             float yaw_vel,
                             float pitch_vel,
                             float roll_vel,
                             float yaw_acc,
                             float pitch_acc,
                             float roll_acc)
{
    char str_buf[MAX_TXT_LINE_LEN] = { 0 };

    BSP_LCD_ClearStringLine(Y_POS_STRING_1);
    BSP_LCD_ClearStringLine(Y_POS_STRING_2);
    BSP_LCD_ClearStringLine(Y_POS_STRING_3);

    BSP_LCD_SetTextColor(VALUES_COLOR);
    BSP_LCD_SetFont(&Font16);
    sprintf(str_buf, "An: %.1f; %.1f; %.1f     ", yaw_ang, pitch_ang, roll_ang);
    BSP_LCD_DisplayStringAt(
            X_POS_STRING, Y_POS_STRING_1, (uint8_t *)str_buf, LEFT_MODE);
    sprintf(str_buf, "Ve: %.1f; %.1f; %.1f     ", yaw_vel, pitch_vel, roll_vel);
    BSP_LCD_DisplayStringAt(
            X_POS_STRING, Y_POS_STRING_2, (uint8_t *)str_buf, LEFT_MODE);
    sprintf(str_buf, "Ac: %.1f; %.1f; %.1f     ", yaw_acc, pitch_acc, roll_acc);
    BSP_LCD_DisplayStringAt(
            X_POS_STRING, Y_POS_STRING_3, (uint8_t *)str_buf, LEFT_MODE);
}

/**
 * @brief Gets one element from gui data queue
 * @param frame - pointer to data frame to be displayed
 * @return servise_status_type_t (STATUS_OK = 0)
 * */
static servise_status_type_t gui_queue_data_get(gui_frame_data_t *frame)
{
    BaseType_t op_status   = pdFAIL;
    stat_t     task_status = { 0 };

    op_status = xQueueReceive(queue_gui, frame, QUEUE_GUI_TIMEOUT);
    if (op_status != pdPASS) {
        task_status.status = SYSTEM_FAIL;
        task_status.source = SYSTEM;
        NOTE_ERROR("Can`t receive GUI data from queue_gui;");
    } else {
        task_status.status = STATUS_OK;
    }
    return task_status.status;
}

BaseType_t gui_queue_data_put(gui_frame_data_t *element)
{
    BaseType_t       op_status = pdFAIL;
    gui_frame_data_t dummy     = { 0 };

    op_status = xQueueSend(queue_gui, element, QUEUE_GUI_TIMEOUT);
    if (op_status != pdPASS) {
        op_status = xQueueReceive(queue_gui, &dummy, 0);
        op_status = xQueueSend(queue_gui, element, 0);
        NOTE_ERROR("Can`t put GUI data to queue_gui;");
    }
    return op_status;
}

static void gui_frame(void *args)
{
    gui_frame_data_t frame;
    TickType_t       last_wake_time;
    last_wake_time       = xTaskGetTickCount();
    BaseType_t op_status = pdFAIL;

    while (1) {
        op_status = gui_queue_data_get(&frame);
        if (op_status != pdFAIL) {
            gui_draw_pitch_val(frame.raw_angle);
            gui_draw_border();
            gui_draw_compass_line(frame.raw_angle);
            gui_draw_pry_val(frame.yaw_ang,
                             frame.pitch_ang,
                             frame.roll_ang,
                             frame.yaw_vel,
                             frame.pitch_vel,
                             frame.roll_vel,
                             frame.yaw_acc,
                             frame.pitch_acc,
                             frame.roll_acc);
        }

        vTaskDelayUntil(&last_wake_time, FRAME_PERIOD);
    }
}

void gui_init(void)
{
    /* Initialize the LCD */
    BSP_LCD_Init();

    /* Initialize the LCD Layers */
    BSP_LCD_LayerDefaultInit(MAIN_LCD_LAYER, LCD_FRAME_BUFFER);

    /* Set LCD Foreground Layer  */
    BSP_LCD_SelectLayer(MAIN_LCD_LAYER);

    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    /* Clear the LCD */
    BSP_LCD_SetBackColor(BACKGROUND_COLOR);
    BSP_LCD_Clear(BACKGROUND_COLOR);

    /* Set the LCD Text Color */
    BSP_LCD_SetTextColor(MANE_STRING_COLOR);

    gui_set_compass_line(LINE_COLOR, LINE_LENGTH);
}

gui_status_t gui_start(uint16_t queue_size)
{
    BaseType_t op_status = pdFAIL;

    //==========================================================================QUEUE creation
    queue_gui = xQueueCreate(queue_size, sizeof(gui_frame_data_t));

    if (queue_gui == NULL) {
        NOTE_ERROR("Can`t create the queue for the GUI data;");
        return (GUI_FAIL);
    }
    NOTE_INFO("The GUI is initialized and running;");

    //===========================================================================TASK creation
    op_status = xTaskCreate(gui_frame,
                            "task_gui_frame_draw",
                            (OPTIMAL_STACK_SIZE),
                            NULL,
                            osPriorityBelowNormal2,
                            &task_gui_frame);

    if (op_status != pdPASS) {
        return (GUI_FAIL);
    }
    return GUI_OK;
}
