/*
 * gui.c
 *
 *  Created on: Jul 7, 2025
 *      Author: andrii-vukolov
 */
#include "gui.h"
#include "ili9341.h"
#include "stm32f429i_discovery_lcd.h"
#include "lcd.h"
#include <math.h>
#include "main.h"
#include <stdio.h>
//#include "../../../Utilities/Fonts/fonts.h"
//#include "../../../Utilities/Fonts/font24.c"
//#include "../../../Utilities/Fonts/font20.c"
//#include "../../../Utilities/Fonts/font16.c"
//#include "../../../Utilities/Fonts/font12.c"
//#include "../../../Utilities/Fonts/font8.c"

static TaskHandle_t     task_gui_frame;
static gui_frame_data_t gui_frame_data = { 0 };

void gui_draw_raw(double degrees)
{
    line_t   ln            = { 0 };
    uint32_t cx            = BSP_LCD_GetXSize() / 2;
    uint32_t cy            = BSP_LCD_GetYSize() / 2;
    uint32_t lenn          = 50;
    double   angle_radians = degrees * M_PI / 180.0;

    ln.x1 = cx;
    ln.y1 = cy + cy / 2;
    ln.x2 = ln.x1 + (lenn * sin(angle_radians));
    ln.y2 = ln.y1 + (lenn * cos(angle_radians));

    BSP_LCD_DrawLine(ln.x1, ln.y1, ln.x2, ln.y2);
}

void gui_draw_pitch_val(double degrees)
{
    uint8_t str_buf[60] = { 0 };

    sprintf((char *)str_buf, "YAW ANGLE: %2.2f", degrees);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 120, str_buf, CENTER_MODE);
}

void gui_clear(void)
{
    BSP_LCD_Clear(LCD_COLOR_WHITE);
}

void gui_draw_border(void)
{
    BSP_LCD_DrawHLine(0, (BSP_LCD_GetYSize() / 2), BSP_LCD_GetXSize());
}

void gui_draw_pry_val(float yaw_ang,
                      float pitch_ang,
                      float roll_ang,
                      float yaw_vel,
                      float pitch_vel,
                      float roll_vel,
                      float yaw_acc,
                      float pitch_acc,
                      float roll_acc)
{
    char str_buf[60] = { 0 };

    BSP_LCD_SetFont(&Font16);
    sprintf(str_buf, "%.1f; %.1f; %.1f", yaw_ang, pitch_ang, roll_ang);
    BSP_LCD_DisplayStringAt(10, 10, (uint8_t *)str_buf, LEFT_MODE);
    sprintf(str_buf, "%.1f; %.1f; %.1f", yaw_vel, pitch_vel, roll_vel);
    BSP_LCD_DisplayStringAt(10, 30, (uint8_t *)str_buf, LEFT_MODE);
    sprintf(str_buf, "%.1f; %.1f; %.1f", yaw_acc, pitch_acc, roll_acc);
    BSP_LCD_DisplayStringAt(10, 50, (uint8_t *)str_buf, LEFT_MODE);
}

void gui_frame(void *args)
{
    gui_frame_data_t *frame = NULL;
    frame                   = (gui_frame_data_t *)args;
    static float q          = -1.0;

    while (1) {
        gui_clear();
        gui_draw_border();
        gui_draw_pitch_val(frame->raw_angle);
        gui_draw_raw(frame->raw_angle);
        gui_draw_pry_val(frame->yaw_ang,
                         frame->pitch_ang,
                         frame->roll_ang,
                         frame->yaw_vel,
                         frame->pitch_vel,
                         frame->roll_vel,
                         frame->yaw_acc,
                         frame->pitch_acc,
                         frame->roll_acc);
        gui_frame_data.yaw_ang   = gui_frame_data.yaw_ang + (5.0 * q);
        gui_frame_data.pitch_ang = gui_frame_data.pitch_ang + (5.0 * q);
        gui_frame_data.roll_ang  = gui_frame_data.roll_ang + (5.0 * q);
        gui_frame_data.yaw_vel   = gui_frame_data.yaw_vel + (5.0 * q);
        gui_frame_data.pitch_vel = gui_frame_data.pitch_vel + (5.0 * q);
        gui_frame_data.roll_ang  = gui_frame_data.roll_ang + (5.0 * q);
        gui_frame_data.yaw_acc   = gui_frame_data.yaw_acc + (5.0 * q);
        gui_frame_data.pitch_acc = gui_frame_data.pitch_acc + (5.0 * q);
        gui_frame_data.roll_acc  = gui_frame_data.roll_acc + (5.0 * q);
        if (gui_frame_data.raw_angle < 359)
            gui_frame_data.raw_angle = gui_frame_data.raw_angle + (5.0);
        else
            gui_frame_data.raw_angle = 0;
        q = (-q);
    }
    vTaskDelay(200 / configTICK_RATE_HZ); //5 fps
}

void gui_init(void)
{
    /* Initialize the LCD */
    BSP_LCD_Init();

    /* Initialize the LCD Layers */
    BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);

    /* Set LCD Foreground Layer  */
    BSP_LCD_SelectLayer(1);

    BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

    /* Clear the LCD */
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    /* Set the LCD Text Color */
    BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

    /* Display LCD messages */
    BSP_LCD_SetFont(&Font8);
    BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F429I BSP", CENTER_MODE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

    BSP_LCD_DrawHLine(0, (BSP_LCD_GetYSize() / 2), BSP_LCD_GetXSize());

    gui_frame_data.yaw_ang   = 60;
    gui_frame_data.pitch_ang = 45;
    gui_frame_data.roll_ang  = 30;
    gui_frame_data.yaw_vel   = 5.1;
    gui_frame_data.pitch_vel = 6.8;
    gui_frame_data.roll_ang  = -8.0;
    gui_frame_data.yaw_acc   = 0.1;
    gui_frame_data.pitch_acc = 0.8;
    gui_frame_data.roll_acc  = 9.8;
    gui_frame_data.raw_angle = 50;
}

gui_status_t gui_start(void)
{
    BaseType_t op_status = pdFAIL;

    //===========================================================================TASK creation
    op_status = xTaskCreate(gui_frame,
                            "task_gui_frame_draw",
                            (OPTIMAL_STACK_SIZE),
                            (void *)&gui_frame_data,
                            osPriorityBelowNormal2,
                            &task_gui_frame);

    if (op_status != pdPASS) {
        return (GUI_FAIL);
    }
    return GUI_OK;
}
