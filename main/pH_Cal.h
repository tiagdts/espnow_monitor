/*
 * pH_Cal.h
 *
 *  Created on: Sep 9, 2024
 *      Author: dschmidt
 */

#ifndef MAIN_PH_CAL_H_
#define MAIN_PH_CAL_H_
// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

//#include "PCAL9554C.h"
//#include "HD44780.h"
#include "espnow_.h"

#include "io.h"
//#include "Sleep_Task.h"
//#include  "polyfit.h"


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
} state_t;

bool run_task( void );
bool calibrating( void );
void set_calibration_active( void );
void clr_calibration_active( void );
void calibration_Task(void *pvParameter);
void set_button( enum button_condition button);

#define CAL_START		0
#define MID_STANDARD	1
#define LOW_STANDARD	2
#define HIGH_STANDARD	3
#define CAL_PASS		4
#define CAL_FAIL		5
#define CAL_COEFF		6
#define CAL_END			7
#define CAL_RESULTS		8

#define INTERCEPT_COEFF	2
#define SLOPE_COEFF		1
#define EXPONENT_COEFF	0
#define PH_ARRAY_SIZE 	16


#endif /* MAIN_PH_CAL_H_ */
