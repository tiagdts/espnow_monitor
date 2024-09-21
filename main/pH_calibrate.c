/*
 * pH_calibrate.c
 *
 *  Created on: Sep 21, 2024
 *      Author: tiagd
 */

#include "pH_calibrate.h"

static char message[11][20] = 	{
									"OFF:Press btn",
									"Requesting Cal",
									"Request Timed Out",
									"Mid Std:",
									"Low Std:",
									"High Std:",
									"Calibration Failed",
									"Success",
									"Coefficients",
									"FINISHED <Btn>",
									"Results"
								};

static QueueHandle_t gpio_evt_queue = NULL;

void clearScreen( void )
{
	char str[] ="                    ";
	LCD_setCursor(0, 0);
	LCD_writeStr(str);
	LCD_setCursor(0, 1);
	LCD_writeStr(str);
	LCD_setCursor(0, 2);
	LCD_writeStr(str);
	LCD_setCursor(0, 3);
	LCD_writeStr(str);

}
void IRAM_ATTR pH_cal_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void update_display(char* message, uint8_t line, uint8_t col )
{
	LCD_setCursor(col, line);
	LCD_writeStr(message);
}

void pH_cal_attach_interrupts( void )
{

    // install ISR service with default configuration
	ESP_ERROR_CHECK( gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT) );

	// attach interrupt service routine for BUTTON_PRESS
	ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PRESS, pH_cal_isr_handler, (void*) BUTTON_PRESS ));

}


void pH_cal_init_queue( void )
{
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

}


void calibration_Task(void *pvParameter)
{
	state_t cal_state = CAL_OFF;
	//message_t send_str = MSG_REQUEST;
	bool first = true;
	uint32_t io_num;

	pH_cal_init_queue( );
	pH_cal_attach_interrupts( );

	while(1)
	{
		// check for status change
		if(xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_PERIOD_MS))
		{
			switch(io_num)
			{
				case BUTTON_PRESS :
						cal_state += 1;
						if( cal_state >= CAL_SHUTDOWN )
						{
						 	LCD_clearScreen( );
						 	LCD_home();
							cal_state = CAL_OFF;
						}
						first = true;
					break;

				default :
					break;
			}
		}

		switch( cal_state )
		{

			case CAL_OFF:
					if( first )
					{
						//update display
						update_display(message[MSG_OFF], 0, 0);
						first = false;
					}
				break;

			case CAL_INIT:
					if( first )
					{
						//update display
						update_display(message[MSG_REQUEST], 0, 0);
						first = false;
					}
				break;

			case CAL_MID_POINT:
					if( first )
					{
						//update display
						update_display(message[MSG_MID_POINT], 1, 0);
						first = false;
					}
				break;

			case CAL_LOW_POINT:
					if( first )
					{
						//update display
						update_display(message[MSG_LOW_POINT], 2, 0);
						first = false;
					}
				break;

			case CAL_HIGH_POINT:
					if( first )
					{
						//update display
						update_display(message[MSG_HIGH_POINT], 3, 0);
						first = false;
					}
				break;

			case CAL_REGRESSION:

				break;

			case CAL_WAIT:
				break;

			case CAL_SHUTDOWN:
					if( first )
					{
					 	clearScreen( );
					 	LCD_home();
					 	vTaskDelay(2000 / portTICK_PERIOD_MS);
						//update display
						update_display(message[MSG_COEFS], 0, 0);
						first = false;
					}
				break;

			default:
				break;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
