/*
 * log_service.h
 *
 *  Created on: Jul 1, 2025
 *      Author: andrii-vukolov
 */

#ifndef INC_LOG_SERVICE_H_
#define INC_LOG_SERVICE_H_

#include "logging_levels.h"
#include "FreeRTOS.h"

#ifndef LIBRARY_LOG_NAME
#define LIBRARY_LOG_NAME "DEMO-DEVICE"
#endif

#ifndef LIBRARY_LOG_LEVEL
#define LIBRARY_LOG_LEVEL LOG_INFO
#endif

/**
 * @brief Function for store logging data to log_buffer according to LIBRARY_LOG_LEVEL
 * @param *log_msg - text string max length LOG_MSG_SIZE *
 * */
void func_logger(char *log_msg);

/**
 * @brief Function for store measured data to log_buffer
 * @param *elt - pointer to queue element contain measured data
 * */
void func_logger_data(queue_data_element_t *elt);

#ifndef SdkLog
#define SdkLog(message) func_logger message
#endif

#include "logging_stack.h"

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
