/*
 * pH_calibrate.c
 *
 *  Created on: Sep 21, 2024
 *      Author: tiagd
 */

#include "pH_calibrate.h"

static char message[12][20] = 	{
									"OFF: Press <Btn>",
									"Requesting Cal",
									"Request Timed Out",
									"Next Standard->Btn",
									"Mid Std:",
									"Low Std:",
									"High Std:",
									"Calibration Failed",
									"Success",
									"Coefficients <Btn>",
									"FINISHED <Btn>",
									"Results"
								};

static QueueHandle_t gpio_evt_queue = NULL;

static time_t button_time = 0;
static int16_t button_data = -1;
static time_t last_button_time = 0;


enum button_condition button_state = IDLE;

void set_button( enum button_condition button)
{
	button_state = button;
}

void send_button_press( int16_t data )
{
	buttonData_t button;

	button_time += 1;
	last_button_time = button_time;
	button.time	= button_time;
	button.location_id = POND;
	button.button_type = BUTTON_TYPE_PH_CAL;
	button.button_data = data;
	button_data = data;
	printf( "button Data %d\n",data );


	updateButtonloc( &button );

}

bool get_button_press( void )
{
	buttonData_t data;
	if( updateButton( &data ) == DATA_READ )
	{
		// any start button will do
		if( data.button_data == START ) return( true );
		if( (data.time == last_button_time) && (data.button_data == button_data ) )
		{
			return( true );
		}
	}
	return( false );
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

void display_dot( bool *dot_on, uint8_t line, uint8_t col)
{
	LCD_setCursor(col, line);
	if( *dot_on )
	{
		*dot_on = false;
		LCD_writeChar('.');
	}
	else
	{
		LCD_writeChar(' ');
		*dot_on = true;
	}
}


void calibration_Task(void *pvParameter)
{
	uint16_t count = 0;
	state_t cal_state = CAL_OFF;
	state_t last_state = CAL_OFF;
	state_t next_state = CAL_OFF;
	//client_state_t client = OFF;
	bool first = true;
	bool calibration_started  = false;
	bool dot_on = true;
	//bool step_complete = false;
	bool button_acknowledge = false;
	uint32_t io_num;
	uint32_t incomingStatus;
	pHCalData_t calibration_data;
	char dataStr[20];

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
						printf("Button Press\n");
						if( next_state == CAL_OFF )
							cal_state += 1;
						else if( last_state == CAL_SHUTDOWN )
							cal_state = CAL_OFF;
						else if( last_state == CAL_INIT ) cal_state = next_state;
						// if the last button press was not acknowledge ignore this button press
						else if( button_acknowledge )
						{
							button_acknowledge = false;
							cal_state = next_state;
						}
/*
						if( cal_state >= CAL_SHUTDOWN )
						{
						 	LCD_clearScreen( );
						 	LCD_home();
							cal_state = CAL_OFF;
						}
*/
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
					// send cal request until the pH calibration data starts to be sent
					// from the pond monitor
					if( !calibration_started )
					{
						send_button_press( START_BTN );
						display_dot(&dot_on, 0, strlen(message[MSG_REQUEST])+1);
					}
					else
					{
						cal_state = CAL_WAIT;
						last_state = CAL_INIT;
						first = true;
					}
				break;

			case CAL_MID_POINT:
					if( first )
					{
						//send_button_press( NEXT_BTN );
						//update display
						dot_on = true;
						update_display(message[MSG_MID_POINT], 1, 0);
						first = false;
					}
					LCD_setCursor(10,1);
					if( ( calibration_data.time != 0 ) && ( calibration_data.state == MID_POINT ) )
					{
						sprintf(dataStr, "%1.4f", calibration_data.pH_volts_mid);
						LCD_writeStr(dataStr);
						cal_state = CAL_WAIT;
						last_state = CAL_MID_POINT;
						first = true;

					}
					else
					{
						display_dot(&dot_on, 1, strlen(message[MSG_MID_POINT])+1);
					}


				break;

			case CAL_LOW_POINT:
					if( first )
					{
						send_button_press( NEXT_BTN );
						//update display
						dot_on = true;
						update_display(message[MSG_LOW_POINT], 2, 0);
						first = false;
					}
					LCD_setCursor(10,2);
					if( ( calibration_data.time != 0 ) && ( calibration_data.state == LOW_POINT ) )
					{
						sprintf(dataStr, "%1.4f", calibration_data.pH_volts_low );
						LCD_writeStr(dataStr);
						cal_state = CAL_WAIT;
						last_state = CAL_LOW_POINT;
						first = true;
					}
					else
					{
						display_dot(&dot_on, 2, strlen(message[MSG_LOW_POINT])+1);
					}

				break;

			case CAL_HIGH_POINT:
				if( first )
				{
					send_button_press( NEXT_BTN );
					//update display
					dot_on = true;
					update_display(message[MSG_HIGH_POINT], 3, 0);
					first = false;
				}
				LCD_setCursor(10,3);
				if( ( calibration_data.time != 0)  && ( calibration_data.state == LOW_POINT ) )
				{
					sprintf(dataStr, "%1.4f", calibration_data.pH_volts_high );
					LCD_writeStr(dataStr);
					first = true;
					cal_state = CAL_REGRESSION;
				}
				else
				{
					display_dot(&dot_on, 3, strlen(message[MSG_HIGH_POINT])+1);
				}

				break;

			case CAL_REGRESSION:
					if(first)
					{
					 	LCD_clearScreen( );
					 	LCD_home();
					 	if( calibration_data.coeff_exp == -100 )
					 	{  // calibration failed
					 		update_display(message[MSG_FAIL], 0, 0);
					 		update_display(message[MSG_FINISHED], 1, 0);
							cal_state = CAL_SHUTDOWN;
							last_state = CAL_REGRESSION;
							first = true;
					 	}
					 	else
					 	{
					 		update_display(message[MSG_COEFS], 0, 0);

					 		sprintf(dataStr,"Exp: %2.5f", calibration_data.coeff_exp);
					 		update_display(dataStr, 1, 0);

					 		sprintf(dataStr,"Slope: %2.5f", calibration_data.coeff_slope);
					 		update_display(dataStr, 2, 0);

					 		sprintf(dataStr,"Intercept: %2.5f", calibration_data.coeff_intercept);
					 		update_display(dataStr, 3, 0);

							cal_state = CAL_WAIT;
							last_state = CAL_REGRESSION;
							first = true;

					 	}

					}


				break;

			case CAL_WAIT:
					if( first )
					{
						first = false;

						switch( last_state )
						{
							case CAL_OFF:

							break;

							case CAL_INIT:
									update_display(message[MSG_NEXT_STANDARD], 0, 0);
									next_state = CAL_MID_POINT;
								break;

							case CAL_MID_POINT:
									update_display(message[MSG_NEXT_STANDARD], 0, 0);
									next_state = CAL_LOW_POINT;
								break;

							case CAL_LOW_POINT:
									update_display(message[MSG_NEXT_STANDARD], 0, 0);
									next_state = CAL_HIGH_POINT;
								break;

							case CAL_REGRESSION:
								break;

							case CAL_HIGH_POINT:
								break;

							case CAL_WAIT:
								break;

							case CAL_SHUTDOWN:
								break;


						}
					}
					else
					{
						if( last_state == CAL_REGRESSION )
							display_dot(&dot_on, 0, strlen(message[MSG_COEFS])+1);
						else display_dot(&dot_on, 0, strlen(message[MSG_NEXT_STANDARD])+1);
					}
				break;

			case CAL_SHUTDOWN:
					if( first )
					{
					 	LCD_clearScreen( );
					 	LCD_home();
					 	vTaskDelay(1000 / portTICK_PERIOD_MS);
						//update display
						update_display(message[MSG_FINISHED], 0, 0);
						first = false;
						last_state = CAL_SHUTDOWN;
					}

				break;

			default:
				break;
		}

		// check for incoming messages
		incomingStatus = getDataReadyStatus( );
		if( incomingStatus != NO_DATA_RDY )
		{
			// check for calibration  data update
			if( ( incomingStatus & PH_CAL_DATA_RDY ) == PH_CAL_DATA_RDY )
			{
				if( !calibration_started )
				{
					calibration_started = true;
				}
				// get calibration data
				else
				{
					updatepHCal( &calibration_data );
				}
			}

			// check for returned button press data
			if( ( incomingStatus & BUTTON_DATA_RDY ) == BUTTON_DATA_RDY )
			{
				if( get_button_press( ) )
				{
					printf("Button Press Acknowledged\n");
					button_acknowledge = true;
				}
				else
				{
					printf("Button Not Acknowledged\n");
					button_acknowledge = false;
				}
			}
		}
		if(++count == 10 )
		{
			count = 0;
			printf("State:%d, Last_state:%d, Next state:%d, btn ack:%d\n",cal_state, last_state, next_state,button_acknowledge);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

