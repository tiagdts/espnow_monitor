/*
 * pH_calibrate.h
 *
 *  Created on: Sep 21, 2024
 *      Author: tiagd
 */

#ifndef PH_CALIBRATE_H_
#define PH_CALIBRATE_H_
// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

#include "espnow_.h"
#include "io.h"
#include "HD44780.h"

#define ESP_INTR_FLAG_DEFAULT 0

typedef enum
{
	CAL_OFF,
	CAL_MODE_INIT,
	CAL_MID_POINT,
	CAL_LOW_POINT,
	CAL_HIGH_POINT,
	CAL_REGRESSION,
	CAL_WAIT,
	CAL_SHUTDOWN
} cal_state_t;

/*
typedef enum
{
	CAL_OFF,
	CAL_INIT,
	CAL_MID_POINT,
	CAL_LOW_POINT,
	CAL_HIGH_POINT,
	CAL_REGRESSION,
	CAL_WAIT,
	CAL_SHUTDOWN
} state_t;

typedef enum
{
	OFF,
	CAL_MODE_INIT,
	MID_POINT,
	LOW_POINT,
	HIGH_POINT,
	REGRESSION,
	WAIT,
	SHUTDOWN
} client_state_t;
*/
typedef enum
{
	MSG_COLLECTING,
	MSG_OFF,
	MSG_REQUEST,
	MSG_REQUEST_TIME_OUT,
	MSG_NEXT_STANDARD,
	MSG_MID_POINT,
	MSG_LOW_POINT,
	MSG_HIGH_POINT,
	MSG_FAIL,
	MSG_PASS,
	MSG_COEFS,
	MSG_FINISHED,
	MSG_RESULTS,
	MSG_BLANK,
	MSG_BLANK2

} message_t;

void calibration_Task(void *pvParameter);

#endif /* PH_CALIBRATE_H_ */
