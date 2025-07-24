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

#define RAW_LENGTH            60
#define QUEUE_GUI_DATA_LENGTH 10
#define FRAME_PERIOD          50

typedef enum { GUI_OK, GUI_FAIL } gui_status_t;

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

typedef struct {
    float         val_x;
    float         val_y;
    float         val_z;
    uint32_t      timestamp;
    data_source_t source;
} gui_data_element_t;

/**
 * @brief Draws row at the center of bottom of lcd turned in
 * @param degrees - angle degrees between raw vector the nord direction
 * */
void gui_draw_raw(double degrees);

/**
 * @brief Draws text string with angle value of raw direction
 * @param degrees - angle value to draw
 * */
void gui_draw_pitch_val(double degrees);

/**
 * @brief Clears whole display (fills it by background color) *
 * */
void gui_clear(void);

/**
 * @brief Draws horizontal line in the middle of display
 * */
void gui_draw_border(void);

/**
 * @brief Clears current pitch value
 * */
void gui_clear_pitch_val(void);

/**
 * @brief Clears current pitch/roll/yaw values
 * */
void gui_clear_pry(void);

/**
 * @brief Clears current raw
 * */
void gui_clear_last_raw(void);

/**
 * @brief Set params of raw to be drawn
 * */
void gui_set_raw(uint32_t color, uint32_t len);

/**
 * @brief calculates current angles based on gyroscope and accelerometer data.
 * Prints current angle and velocity of angle change at the top of screen
 * @param data - pointer to structure contained data from gyroscope queue
 * */
void gui_draw_gyro_data(const queue_data_element_t *data);

/**
 * @brief Prints last accelerometer data
 * @param data - pointer to structure contained queue elements of accelerometer
 * */
void gui_draw_accel_data(const queue_data_element_t *data);

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
 * @brief Gets one element from gui data queue
 * @param frame - pointer to data frame to be displayed
 * @return servise_status_type_t (STATUS_OK = 0)
 * */
servise_status_type_t gui_queue_data_get(gui_frame_data_t *frame);

/**
 * @brief Puts one element to gui data queue
 * @param element - pointer to data element to be placed into gui data queue
 * @return BaseType_t (pdFAIL = 0)
 * */
BaseType_t gui_queue_data_put(gui_data_element_t *element);

#endif /* INC_GUI_H_ */
