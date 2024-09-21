/*
 * pH_Cal.c
 *
 *  Created on: Sep 9, 2024
 *      Author: dschmidt
 */

#include "pH_Cal.h"
#include "ADS1115.h"

static double x[3] = { 4.01, 6.86, 9.18}; // Calibration Standards
static double y[3]; // pH volts
static double coeff[3];
enum button_condition button_state = IDLE;

static QueueHandle_t gpio_evt_queue = NULL;

static bool calibration_active = false;

static char message[10][20] = {"pH Calibration <Btn>",
							"Mid Std:",
							"Low Std:",
							"High Std:",
							"Calibration Failed",
							"Success",
							"Coefficents",
							"FINISHED <Btn>",
							"Results"			
							};

static float pH_data[PH_ARRAY_SIZE];
static uint8_t valid_data[PH_ARRAY_SIZE];


static float calc_mean( void )
{
	uint16_t i;
	float sum, sum_count;
	float min, max;
	uint16_t smallest;
	uint16_t largest;
	float mean = 0;

	// make all valid
	memset(valid_data, 1, sizeof( valid_data[0] )* PH_ARRAY_SIZE);

	// determine smallest and largest for each sensor
	min = pH_data[0];
	max = pH_data[0];
	smallest = 0;
	largest = 0;
	for(i = 1; i < PH_ARRAY_SIZE; i++)
	{

		// check for smallest
		if( pH_data[i] < min )
		{
			min = pH_data[i];
			smallest = i;
		}
		// check for largest
		if( pH_data[i] > max )
		{
			max = pH_data[i];
			largest = i;
		}

	}
	valid_data[ smallest ] = 0; // mark smallest invalid
	valid_data[ largest ]  = 0;  // mark largest invalid

	// calculate average excluding the smallest and the largest readings

	sum = 0;
	sum_count = 0;
	for(i = 0; i < ARRAY_SIZE; i++)
	{
		if( valid_data[i] )
		{
			sum += pH_data[i];
			sum_count++;
		}
	}
	if(sum_count>0)
		mean = sum/sum_count;
	else mean = 0;

	printf("Min: %1.4f, Max: %1.4f, Mean: %1.4f\n", min, max, mean);
	return mean;
}

void set_button( enum button_condition button)
{
	button_state = button;
}

bool run_task( void )
{
	return calibration_active;
}

bool calibrating( void )
{
	return calibration_active;
}

void set_calibration_active( void )
{
	calibration_active = true;
}

void clr_calibration_active( void )
{
	calibration_active = false;
}

void IRAM_ATTR pH_cal_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

#ifdef LCD_ATTACHED
void display_results( void )
{
	char tmp_str[40];
	int8_t i;
	
	LCD_clearScreen();
	LCD_setCursor(10,0);
	LCD_writeStr( message[CAL_RESULTS] );
	
	for(i=0;i<3;i++)
	{
		//sprintf(tmp_str, "x%d:%1.2lf,y%d:%1.2lf,Coeff%d:%1.5lf",i,x[i],i,y[i],i,coeff[i]);
		sprintf(tmp_str, "%d,%1.2lf,%1.2lf,%1.5lf",i,x[i],y[i],coeff[i]);
		printf("%s\n", tmp_str );
		LCD_setCursor(0,i+1);
		LCD_writeStr( tmp_str );
	}
}

void update_display(char* message, uint8_t line, bool *first, bool display_volts, double* volts )
{
	uint8_t col;
	//float volts;
	char volt_str[10];
	
	if(*first==true)
	{
		*first = false;
		LCD_setCursor(0, line);
		LCD_writeStr(message);
	}
	if(display_volts)
	{
		col = strlen( (const char*)(message) ) + 1;
		LCD_setCursor(col, line);
		*volts = get_pH_volts( );
		sprintf(volt_str,"%1.5lf", *volts );
		LCD_writeStr( volt_str);
	}
}
#endif

void initialize_calibration( void )
{
	TaskHandle_t handle_ADS1115_task = NULL;
	//bool first = true;

 	//calibration_active = true;
 	// this task runs to get pH voltages during calibration, task is terminated when calibration is complete
	xTaskCreate(&pH_Cal_ADS1115_Task, "ADS1115_Task", 2048, NULL, 2, &handle_ADS1115_task );

}

void terminate_calibration( void )
{

	
	// clear the bit controlled by the button press circuit
	force_cal_mode_off( );
	
	// clear the calibration indicator in the sleep task
	// so normal logging can resume
	//clr_Calibration_Request( );
	
	// terminate calibration task
	calibration_active = false;
	clr_Calibration_Request( ); // tell sleep_Task to resume normal operation
}

void pH_cal_attach_interrupts( void )
{
	// attach interrupt service routine for BUTTON_PRESS
    // install ISR service with default configuration
//	ESP_ERROR_CHECK( gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT) );
	ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PRESS, pH_cal_isr_handler, (void*) BUTTON_PRESS ));

}


void pH_cal_init_queue( void )
{
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));


}


void calibration_Task(void *pvParameter)
{
	
	
	
	int16_t calibration_state = 0;
	state_t cal_state = OFF;
	//state_t last_state = OFF;
	uint32_t io_num;
	//bool first = true;
	uint32_t count = 0;

	pH_cal_init_queue();
	pH_cal_attach_interrupts();
	
	pHCalData_t cal_data;
	
	time_t timestamp;
	//bool data_updated = false;
	//float pH_volts = 0;
	uint8_t temp_status;
	enum button_condition last_button_state = IDLE;

	memset( &cal_data, 0, sizeof( pHCalData_t ) );

	while(1)
	{
		if( calibration_active )
		{
			// check for status change from hardware button
	    	if(xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_PERIOD_MS))
	    	{
				switch(io_num)
				{
					case BUTTON_PRESS :
							calibration_state = gpio_get_level(PH_CAL_MODE);
							printf("Button Press\n");
							// used for updating display at each step
							//first = true;
							count = 0;
							// clear out cal_data
							if( cal_state == OFF )
								memset( &cal_data, 0, sizeof( pHCalData_t ) );
							if( !calibration_state ) cal_state = SHUTDOWN;
								else cal_state += 1; // go to next state
							printf(" State: %d, count: %lu\n", cal_state, count);
						break;
						
					default:
				}
			}

	    	// check status for espnow button
	    	switch( button_state )
	    	{
				case IDLE :  // do nothing
						last_button_state = IDLE;
					break;

				case START :
						if( last_button_state != START )
						{
							last_button_state = START;
							if( !calibration_state ) cal_state = SHUTDOWN;
								else cal_state = MID_POINT; // go to next state
							memset( &cal_data, 0, sizeof( pHCalData_t ) );
							button_state = IDLE;
						}
					break;

				case NEXT :
						if( last_button_state != NEXT )
						{
							last_button_state = NEXT;
							if( !calibration_state ) cal_state = SHUTDOWN;
								else cal_state += 1; // go to next state
							button_state = IDLE;
						}
					break;

				case BACK:
						if( last_button_state != BACK )
						{
							last_button_state = BACK;
							if( !calibration_state ) cal_state = SHUTDOWN;
								else if( cal_state > OFF ) cal_state -= 1; // go to previous state
							button_state = IDLE;
						}
					break;

				case END:
						if( last_button_state != END )
						{
							last_button_state = END;
							cal_state = SHUTDOWN;
							button_state = IDLE;
						}
					break;

				default:
					 button_state = IDLE;
					break;

	    	}


			switch(cal_state)
			{
				case OFF:
					// do nothing

					break;
					
				case CAL_MODE_INIT:
					//last_state = CAL_MODE_INIT;
						// initialize display
						//initialize_calibration();
						cal_state = MID_POINT;
					break;
					
				case MID_POINT:
						ADS1115_get_pH_conversion( 5, &pH_data[count++] );
						cal_data.temp = get_MAX31865_temperature( &temp_status );
						cal_data.state = MID_POINT;
						if( count > PH_ARRAY_SIZE)
						{
							count = 0;
							cal_data.pH_volts_mid = calc_mean( );
							printf("Mid volts %1.4f: \n", cal_data.pH_volts_mid);
							// get sample time
							time(&timestamp);
							cal_data.time = timestamp;
							updatepHCalloc( &cal_data );
						}
						else
						{
							cal_data.time = 0;
							cal_data.pH_volts_mid = pH_data[count-1];
							updatepHCalloc( &cal_data );
						}
					break;

				case LOW_POINT:

						ADS1115_get_pH_conversion( 5, &pH_data[count++] );
						cal_data.temp = get_MAX31865_temperature( &temp_status );
						cal_data.state = LOW_POINT;
						if( count > PH_ARRAY_SIZE)
						{
							count = 0;
							cal_data.pH_volts_low = calc_mean( );
							printf("Low volts %1.4f: \n", cal_data.pH_volts_low);
							// get sample time
							time(&timestamp);
							cal_data.time = timestamp;
							updatepHCalloc( &cal_data );
						}
						else
						{
							cal_data.pH_volts_low = pH_data[count-1];
							cal_data.time = 0;
							updatepHCalloc( &cal_data );
						}
					break;
					
				case HIGH_POINT:

						ADS1115_get_pH_conversion( 5, &pH_data[count++] );
						cal_data.temp = get_MAX31865_temperature( &temp_status );
						cal_data.state = HIGH_POINT;
						if( count > PH_ARRAY_SIZE)
						{
							count = 0;
							cal_data.pH_volts_high = calc_mean( );
							printf("High volts %1.4f: \n", cal_data.pH_volts_high);
							// get sample time
							time(&timestamp);
							cal_data.time = timestamp;
							updatepHCalloc( &cal_data );
						}
						else
						{
							cal_data.pH_volts_high = pH_data[count-1];
							cal_data.time = 0;
							updatepHCalloc( &cal_data );
						}

					break;
					
				case REGRESSION:

						y[0] = cal_data.pH_volts_low;
						y[1] = cal_data.pH_volts_mid;
						y[2] = cal_data.pH_volts_high;

						cal_data.state = REGRESSION;

						int rVal = polyfit( 3, x, y, 3, coeff);
						
  						if( 0 == rVal)
  						{ 
    						// pass: display readings and coefficents
    						//display_results( );
  							cal_data.coeff_exp = coeff[0];
  							cal_data.coeff_slope = coeff[1];
  							cal_data.coeff_intercept = coeff[2];
  							printf("exponenet: %2.5f, slope: %2.5f, intercept: %2.5f\n", coeff[0], coeff[1], coeff[2]);
							// get sample time
							time(&timestamp);
							cal_data.time = timestamp;
							updatepHCalloc( &cal_data );
							setSetupData(CAL_VOLTS_LOW, cal_data.pH_volts_low );
							setSetupData(CAL_VOLTS_MID, cal_data.pH_volts_mid );
							setSetupData(CAL_VOLTS_HIGH, cal_data.pH_volts_high );
							setSetupData(COEFF_0_EXP, cal_data.coeff_exp );
							setSetupData(COEFF_1_SLOPE, cal_data.coeff_slope );
							setSetupData(COEFF_2_INTC, cal_data.coeff_intercept );
							setSetupData(CAL_TEMP, cal_data.temp );
 						}
  						else
  						{
							 // fail: 
							 //first = true;
							 //double dummy;
							// update_display(message[CAL_FAIL], 3, &first, false, &dummy );
						}
						cal_state = WAIT;
  						
					break;
					
				case WAIT:
						// do nothing
						printf(".");
					break;
					
				case SHUTDOWN:
						terminate_calibration( );
					break;
					
				default:
					
			}
		}
		else cal_state = OFF;
		
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
	
