/*
 * gui.h
 *
 *  Created on: Jul 7, 2025
 *      Author: andrii-vukolov
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "sensor_service.h"

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

/**
 * @brief Draws row at the center of bottom of lcd turned in
 * @param degrees - angle degrees between raw vector the nord direction
 * */
void gui_draw_raw(double degrees);

void gui_draw_pitch_val(double degrees);

void gui_clear(void);

void gui_draw_border(void);

void gui_clear_pitch_val(void);

void gui_clear_pry(void);

void gui_clear_last_raw(void);

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

void gui_init(void);

gui_status_t gui_start(void);

#endif /* INC_GUI_H_ */
