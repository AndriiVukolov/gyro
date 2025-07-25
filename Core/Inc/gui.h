/*
 * gui.h
 *
 *  Created on: Jul 7, 2025
 *      Author: andrii-vukolov
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "sensor_service.h"
#include "stm32f429i_discovery_lcd.h"

#define LINE_LENGTH           60
#define LINE_COLOR            LCD_COLOR_RED
#define VALUES_COLOR          LCD_COLOR_DARKGREEN
#define MANE_STRING_COLOR     LCD_COLOR_DARKBLUE
#define BACKGROUND_COLOR      LCD_COLOR_WHITE
#define QUEUE_GUI_DATA_LENGTH 10
#define FRAME_PERIOD          150
#define QUEUE_GUI_TIMEOUT     100
#define MAIN_LCD_LAYER        1
#define MAX_TXT_LINE_LEN      60

#define X_POS_STRING      10
#define Y_POS_STRING_1    10
#define Y_POS_STRING_2    30
#define Y_POS_STRING_3    50
#define Y_POS_STRING_4    70
#define Y_POS_STRING_5    90
#define Y_POS_STRING_MAIN 120
#define X_BORDER_POS      0
#define Y_BORDER_POS      ((BSP_LCD_GetYSize() / 2) + 10)

typedef enum { GUI_OK, GUI_FAIL } gui_status_t;

typedef struct {
    float yaw_ang;
    float pitch_ang;
    float roll_ang;
    float yaw_vel;
    float pitch_vel;
    float roll_vel;
    float yaw_acc;
    float pitch_acc;
    float roll_acc;
    float raw_angle;
} gui_frame_data_t;

//typedef struct {
//    float         val_x;
//    float         val_y;
//    float         val_z;
//    uint32_t      timestamp;
//    data_source_t source;
//} gui_data_element_t;

/**
 * @brief inits gui
 * */
void gui_init(void);

/**
 * @brief Creates queue for data frame and task for displaying one and starts gui service
 * @param queue_size - size of queue for gui data
 * @return gui_status_t (GUI_OK = 0, GUI_FAIL = 1)
 * */
gui_status_t gui_start(uint16_t queue_size);

/**
 * @brief Puts one element to gui data queue
 * @param element - pointer to data element to be placed into gui data queue
 * @return BaseType_t (pdFAIL = 0)
 * */
BaseType_t gui_queue_data_put(gui_frame_data_t *element);

#endif /* INC_GUI_H_ */
