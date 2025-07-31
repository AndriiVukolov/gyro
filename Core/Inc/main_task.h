/**
 * @file main_task.h
 * @brief Main module
 * This module is for maintenance data transfer from sensors service to GUI service
 *
 *  Created on: Jul 24, 2025
 *  @author: Andrii V.
 *  @copyright (C) 2025 Alnicko Development OU. All rights reserved.
 */

#ifndef INC_MAIN_TASK_H_
#define INC_MAIN_TASK_H_

#include <stdint.h>
#include "sensor_service.h"
#include "gui.h"

#define TASK_TIMEOUT      (TickType_t)10
#define GYRO_NORMAL_ERROR 0.8

BaseType_t func_main_start(void);

#endif /* INC_MAIN_TASK_H_ */
