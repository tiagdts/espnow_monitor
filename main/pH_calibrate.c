/*
 * pH_calibrate.c
 *
 *  Created on: Sep 21, 2024
 *      Author: tiagd
 */

#include "pH_calibrate.h"

static char message[12][20] = 	{
									"CAL_OFF: Press <Btn>",
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
static buttonData_t buttonIn;
static buttonData_t buttonOut;


button_data_t button_state = IDLE_BTN;

void set_button( button_data_t button)
{
	button_state = button;
}

void copy_in_to_out( void )
{
	buttonOut.time	= buttonIn.time;
	buttonOut.location_id = buttonIn.location_id;
	buttonOut.button_type = buttonIn.button_type ;
	buttonOut.button_data = buttonIn.button_data;
	buttonOut.state = buttonIn.state ;
	buttonOut.last_state = buttonIn.last_state;
	buttonOut.next_state = buttonIn.next_state;

}

void send_button_press( int16_t data, int16_t state, int16_t last_state, int16_t next_state)
{


	button_time += 1;
	last_button_time = button_time;
	buttonOut.time	= button_time;
	buttonOut.location_id = POND;
	buttonOut.button_type = BUTTON_TYPE_PH_CAL;
	buttonOut.button_data = data;
	buttonOut.state = state;
	buttonOut.last_state = last_state;
	buttonOut.next_state = next_state;
	button_data = data;
	printf( "button Data %d\n",data );
	updateButtonloc( &buttonOut );

}

bool get_button_press( void )
{
	if( updateButtonIn( &buttonIn ) == DATA_READ )
	{
		// check for a calibration button
		if( buttonIn.button_type == BUTTON_TYPE_PH_CAL )
		{
			// set local variables from button
			return true;
		}
		else return false;
	}
	return false ;
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
	cal_state_t cal_state = CAL_OFF;
	cal_state_t last_state = CAL_OFF;
	cal_state_t next_state = CAL_OFF;
	//client_state_t client = CAL_OFF;
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
						if( cal_state == CAL_OFF )
						{
							// send start button
							send_button_press( START_BTN, cal_state, last_state, next_state );
							buttonIn.button_data = START_BTN;
						}
						else
						{
							//cal_state = next_state; // go to next state
							// let the pond monitor decide if a change is ready to be made
							send_button_press( CHANGE_BTN, cal_state, last_state, next_state );
						}
					break;

				default :
					break;
			}
		}
//////////////////////////

		// send the START_BTN until buttonOut.button_data changes to IDLE_BTN
		if( buttonIn.button_data == START_BTN )
			send_button_press( START_BTN, cal_state, last_state, next_state );

		switch(cal_state)
		{
			case CAL_OFF:
				// do nothing
				printf("Off\n");

				break;

			case CAL_MODE_INIT:



				break;

			case CAL_MID_POINT:

					// display mid volts
					printf("Mid Standard Volts (%1.3f)\n",calibration_data.pH_volts_mid);
				break;

			case CAL_LOW_POINT:

					// display low volts
					printf("Low Standard Volts (%1.3f)\n",calibration_data.pH_volts_low);
				break;

			case CAL_HIGH_POINT:

					// display high volts
					printf("High Standard Volts (%1.3f)\n",calibration_data.pH_volts_high);

				break;

			case CAL_REGRESSION:
					// wait for new data
					vTaskDelay(2000 / portTICK_PERIOD_MS);
						// display pass/fail
						if( calibration_data.coeff_exp == -100.0 )
							printf("Calibration Failed\n");
						else
						{
							// display coefficients
							printf("Coefficients: %1.5f, %1.5f, %1.5f\n", calibration_data.coeff_exp,
									calibration_data.coeff_slope, calibration_data.coeff_intercept );
							// display file save resutls
							if( calibration_data.saved )
								printf("Calibration Data Saved to spiffs\n");
							else printf("Calibration Data Not Saved to spiffs\n");
						}

				break;

			case CAL_WAIT:

					switch(next_state)
					{
						case CAL_MID_POINT:
							printf("Place Probe in Mid Standard (%1.3f)\n",calibration_data.pH_volts_mid);
							break;

						case CAL_LOW_POINT:
							printf("Place Probe in Low Standard (%1.3f)\n",calibration_data.pH_volts_low);
							break;

						case CAL_HIGH_POINT:
							printf("Place Probe in High Standard (%1.3f)\n",calibration_data.pH_volts_high);
							break;

						default :
							break;

					}
					// do nothing
					printf("wait\n");
				break;

			case CAL_SHUTDOWN:

				break;

			default:

		}
///////////////////////////
#ifdef OLD_CALIBRATION
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
					if( ( calibration_data.time != 0 ) && ( calibration_data.state == CAL_MID_POINT ) )
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
					if( ( calibration_data.time != 0 ) && ( calibration_data.state == CAL_LOW_POINT ) )
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
				if( ( calibration_data.time != 0)  && ( calibration_data.state == CAL_LOW_POINT ) )
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
#endif
		// check for incoming messages
		incomingStatus = getDataReadyStatus( );
		if( incomingStatus != NO_DATA_RDY )
		{
			// check for calibration  data update
			if( ( incomingStatus & PH_CAL_DATA_RDY ) == PH_CAL_DATA_RDY )
			{
				updatepHCal( &calibration_data );
			}

			// check for returned button press data
			if( ( incomingStatus & BUTTON_DATA_RDY ) == BUTTON_DATA_RDY )
			{
				if( get_button_press( ) )
				{
					printf("Button Data Received (Set to Idle)\n");
					cal_state = buttonIn.state;
					last_state = buttonIn.last_state;
					next_state = buttonIn.next_state;
					buttonIn.button_data = IDLE_BTN;
					copy_in_to_out( );
					// stop sending the START_BTN
					printf("State:%d, Last_state:%d, Next state:%d, Button Data: %d\n",
							cal_state, last_state, next_state, buttonOut.button_data);

				}
				else
				{
					printf("Button Data not used\n");
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

