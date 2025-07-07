/*
 * log_service.h
 *
 *  Created on: Jul 1, 2025
 *      Author: andrii-vukolov
 */

#include "FreeRTOS.h"
#include "stdarg.h"

#ifndef INC_LOG_SERVICE_H_
#define INC_LOG_SERVICE_H_

#ifndef LIBRARY_LOG_NAME
#define LIBRARY_LOG_NAME "SENSOR_SERVICE"
#endif

typedef enum { LOG_NONE, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG } log_lvl_t;

#ifndef LIBRARY_LOG_LEVEL
#define LIBRARY_LOG_LEVEL LOG_INFO
#endif

/**
 * @brief Function for store logging data to log_buffer according to LIBRARY_LOG_LEVEL
 * @param *log_msg - text string max length LOG_MSG_SIZE *
 * */
void func_logger(log_lvl_t lvl, const char *log_msg, ...);

#ifndef NOTE_ERROR
#define NOTE_ERROR(message, ...) func_logger(LOG_ERROR, message, ##__VA_ARGS__)
#endif

#ifndef NOTE_WARN
#define NOTE_WARN(message, ...) func_logger(LOG_WARN, message, ##__VA_ARGS__)
#endif

#ifndef NOTE_INFO
#define NOTE_INFO(message, ...) func_logger(LOG_INFO, message, ##__VA_ARGS__)
#endif

#ifndef NOTE_DEBUG
#define NOTE_DEBUG(message, ...) func_logger(LOG_DEBUG, message, ##__VA_ARGS__)
#endif
/**
 * @brief Create a task for periodically read data stored in log_buffer and send it to UART
 * @return pdPASS - if task is created and running; pdFAIL - if not. *
 * */
BaseType_t read_log_service_start(void);

/**
 * @brief Transmitt text over UART
 * @param *str - pointer to text string needed to transmit
 * */
void print(const char *str);

#endif /* INC_LOG_SERVICE_H_ */
