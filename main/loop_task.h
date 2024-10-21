/*
 * loop_task.h
 *
 *  Created on: Oct 21, 2024
 *      Author: tiagd
 */

#ifndef LOOP_TASK_H_
#define LOOP_TASK_H_

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "espnow_.h"
#include "Station_Data_Types.h"

#include "HD44780.h"

void loop_task(void *pvParameter);

#endif /* LOOP_TASK_H_ */
