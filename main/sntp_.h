/*
 * sntp_.h
 *
 *  Created on: Jan 1, 2023
 *      Author: tiagd
 */

#ifndef MAIN_SNTP__H_
#define MAIN_SNTP__H_

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"


bool get_sntp(void);
void sntp_task(void *pvParameter);
bool get_sntp_updated( void );
bool get_sntp_busy( void );


#endif /* MAIN_SNTP__H_ */
