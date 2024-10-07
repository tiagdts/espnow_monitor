/*
 * pH_calibrate.c
 *
 *  Created on: Sep 21, 2024
 *      Author: tiagd
 */

#include "pH_calibrate.h"

static char message[15][21] = 	{
									"Collecting Data",
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
									"Results",
									"                   ",
									"					"
								};

static QueueHandle_t gpio_evt_queue = NULL;

static time_t button_time = 0;
static int16_t button_data = -1;
static time_t last_button_time = 0;
static buttonData_t buttonIn;
static buttonData_t buttonOut;
static pondData_t PondData;


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

uint8_t update_display(char* message, uint8_t line, uint8_t col )
{
	uint8_t len;
	LCD_setCursor(col, line);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	len = strlen( message )+1;
	LCD_writeStr(message);
	//len += 1;
	printf("update_display: %s, %u\n", message, len);
	return len;

}

void clear_line( uint8_t y_pos )
{
	LCD_setCursor(0, y_pos);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	//uint8_t len = strlen( message[ MSG_BLANK ] );
	LCD_writeStr(message[ MSG_BLANK]);

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
		LCD_writeChar('_');
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
	//bool calibration_started  = false;
	bool dot_on = true;
	//bool step_complete = false;
	bool button_acknowledge = false;
	uint32_t io_num;
	uint32_t incomingStatus;
	pHCalData_t calibration_data;
	uint8_t x_pos = 0;
	uint8_t sub_x_pos = 0;
	//char dataStr[20];
	char tmp_str[21];

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
							cal_state = CAL_MODE_INIT;
							first = true;
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
		{
			send_button_press( START_BTN, cal_state, last_state, next_state );

		}


		switch(cal_state)
		{
			case CAL_OFF:
				// do nothing
				printf("Off\n");
				if( first )
				{
					//update display
				 	LCD_clearScreen();
					x_pos = update_display(message[MSG_OFF], 0, 0);
					first = false;
				}
				else
				{
					//display_dot(&dot_on, 0, strlen(message[MSG_OFF])+1);
					display_dot( &dot_on, 0, x_pos );
				}

				break;

			case CAL_MODE_INIT:
				printf("Cal init\n");
				if( first )
				{
					memset( &calibration_data, 0, sizeof( calibration_data ) );
					//update display
					clear_line( 0 );
					update_display(message[MSG_REQUEST], 0, 0);
					first = false;
				}
				else
				{
					//display_dot(&dot_on, 0, strlen(message[MSG_OFF])+1);
					printf("Init mode dot\n");
					display_dot( &dot_on, 0, 14 );
				}


				break;

			case CAL_MID_POINT:
					if( first )
					{
						//update display
						clear_line( 0 );
						x_pos = update_display(message[MSG_COLLECTING], 0, 0);
						first = false;
					}
					else
					{
						sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_mid);
						update_display( tmp_str, 1, sub_x_pos );
						display_dot( &dot_on, 1, 19 );
					}
					// display mid volts
					printf("Mid Standard Volts (%1.3f)\n",calibration_data.pH_volts_mid);
				break;

			case CAL_LOW_POINT:
					if( first )
					{
						//update display
						clear_line( 0 );
						x_pos = update_display(message[MSG_COLLECTING], 0, 0);
						first = false;
					}
					else
					{
						sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_low);
						update_display( tmp_str, 2, sub_x_pos );
						display_dot( &dot_on, 2, 19 );
					}
					// display low volts
					printf("Low Standard Volts (%1.3f)\n",calibration_data.pH_volts_low);
				break;

			case CAL_HIGH_POINT:
					if( first )
					{
						//update display
						clear_line( 0 );
						x_pos = update_display(message[MSG_COLLECTING], 0, 0);
						first = false;
					}
					else
					{
						sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_high);
						update_display( tmp_str, 3, sub_x_pos );
						display_dot( &dot_on, 3, 19 );
					}
					// display high volts
					printf("High Standard Volts (%1.3f)\n",calibration_data.pH_volts_high);

				break;

			case CAL_REGRESSION:
					// wait for new data
					vTaskDelay(2000 / portTICK_PERIOD_MS);
						// display pass/fail
						if( calibration_data.coeff_exp == -100.0 )
						{
							printf("Calibration Failed\n");
							update_display(message[MSG_FAIL], 1, 0);
						}
						else
						{
							// display coefficients
							printf("Coefficients: %1.5f, %1.5f, %1.5f\n", calibration_data.coeff_exp,
									calibration_data.coeff_slope, calibration_data.coeff_intercept );
							// display file save resutls
							if( calibration_data.saved )
								printf("Calibration Data Saved to spiffs\n");
							else printf("Calibration Data Not Saved to spiffs\n");

							update_display(message[MSG_RESULTS], 0, 0);

							sprintf(tmp_str,"Exp: %1.6f",calibration_data.coeff_exp);
							update_display(tmp_str, 1, 0);

							sprintf(tmp_str,"Slope: %1.6f",calibration_data.coeff_slope);
							update_display(tmp_str, 2, 0);


							sprintf(tmp_str,"Intercept: %1.6f",calibration_data.coeff_intercept);
							update_display(tmp_str, 3, 0);

						}
						// delay for 5 seconds
						vTaskDelay(5000 / portTICK_PERIOD_MS);


				break;

			case CAL_WAIT:

					switch(next_state)
					{
						case CAL_MID_POINT:
								printf("Place Probe in Mid Standard (%1.3f)\n",calibration_data.pH_volts_mid);
								if( first )
								{
									//update display
									clear_line( 1 );
									x_pos = update_display(message[MSG_NEXT_STANDARD], 0, 0);
									sub_x_pos = update_display(message[MSG_MID_POINT], 1, 0);
									first = false;
								}
								else
								{
									display_dot( &dot_on, 0, x_pos );
									sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_mid);
									update_display( tmp_str, 1, sub_x_pos );
								}
							break;

						case CAL_LOW_POINT:
							printf("Place Probe in Low Standard (%1.3f)\n",calibration_data.pH_volts_low);
							if( first )
							{
								//update display
								clear_line( 2 );
								x_pos = update_display(message[MSG_NEXT_STANDARD], 0, 0);
								sub_x_pos = update_display(message[MSG_LOW_POINT], 2, 0);
								first = false;
							}
							else
							{
								display_dot( &dot_on, 0, x_pos );
								sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_low);
								update_display( tmp_str, 2, sub_x_pos );
							}
							break;

						case CAL_HIGH_POINT:
							printf("Place Probe in High Standard (%1.3f)\n",calibration_data.pH_volts_high);
							if( first )
							{
								//update display
								clear_line( 3 );
								x_pos = update_display(message[MSG_NEXT_STANDARD], 0, 0);
								sub_x_pos = update_display(message[MSG_HIGH_POINT], 3, 0);
								first = false;
							}
							else
							{
								display_dot( &dot_on, 0, x_pos );
								sprintf(tmp_str,"%1.5f",calibration_data.pH_volts_high);
								update_display( tmp_str, 3, sub_x_pos );
							}
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

		// check for incoming messages
		incomingStatus = getDataReadyStatus( );
		if( incomingStatus != NO_DATA_RDY )
		{
			// check for calibration  data update
			if( ( incomingStatus & PH_CAL_DATA_RDY ) == PH_CAL_DATA_RDY )
			{
				updatepHCal( &calibration_data );
				if(calibration_data.state == CAL_REGRESSION )
				{
					// display resutls, calibration complete
					printf("Calibration Complete\n");
					cal_state = CAL_OFF;
				}
			}

			// check for returned button press data
			if( ( incomingStatus & BUTTON_DATA_RDY ) == BUTTON_DATA_RDY )
			{
				if( get_button_press( ) )
				{
					if( ( buttonIn.button_data == IDLE_BTN ) && buttonIn.state == CAL_MODE_INIT )
					{
						printf("Start button not used: button data %d, state:%d\n",
								buttonIn.button_data, buttonIn.state );

					}
					else
					{
						// this is were the local state machine is set to the
						// remote calibration system
						printf("Button Data Received (Set to Idle)\n");
						cal_state = buttonIn.state;
						last_state = buttonIn.last_state;
						next_state = buttonIn.next_state;
						buttonIn.button_data = IDLE_BTN;
						copy_in_to_out( );
						// stop sending the START_BTN
						printf("State:%d, Last_state:%d, Next state:%d, Button Data: %d\n",
								cal_state, last_state, next_state, buttonOut.button_data);
						first = true;
					}

				}
				else
				{
					printf("Button Data not used\n");
				}
			}

			// check for pond data update
			if( ( incomingStatus & POND_DATA_RDY ) == POND_DATA_RDY )
			{
				updatePond( &PondData );
				if(cal_state == CAL_OFF)
				{
					// display resutls
					printf("Pond Data\n");
					sprintf(tmp_str, "pH: %2.2f",PondData.pH);
					update_display(tmp_str, 1, 0);
					sprintf(tmp_str, "Temp: %2.2f",PondData.water_temperature);
					update_display(tmp_str, 2, 0);
					sprintf(tmp_str, "Light:%u[%d]:%lu",PondData.light_level, PondData.hour, PondData.hourly_light_accum);
					update_display(tmp_str, 3, 0);

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

