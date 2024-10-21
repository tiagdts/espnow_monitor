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

#include "string.h"

#include "espnow_.h"
#include "Station_Data_Types.h"

#include "HD44780.h"


// Day of Week
#define SUNDAY		0x01
#define MONDAY		0x02
#define TUESDAY		0x04
#define WEDNESDAY	0x08
#define THURSDAY	0x10
#define FRIDAY		0x20
#define SATURDAY	0x40

#define MSG_BLANK	0

void loop_task(void *pvParameter);

#endif /* LOOP_TASK_H_ */
