/*
 * loop_task.c
 *
 *  Created on: Oct 21, 2024
 *      Author: tiagd
 */
#include "loop_task.h"

static time_t now = 0;
static struct tm *timeinfo;

static char heartbeat[2][3] = {
								"*",
								" "
							};



uint8_t update_display(char* message, uint8_t line, uint8_t col )
{
	uint8_t len;
	LCD_setCursor(col, line);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	len = strlen( message )+1;
	LCD_writeStr(message);
	//len += 1;
	//printf("update_display: %s, %u\n", message, len);
	return len;

}

#ifdef CLEAR_LINE
void clear_line( uint8_t y_pos )
{
	LCD_setCursor(0, y_pos);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	//uint8_t len = strlen( message[ MSG_BLANK ] );
	LCD_writeStr(message[ MSG_BLANK]);

}
#endif

#ifdef RX8804_TIME
uint8_t RX8804_time_from_tm(struct tm *timeinfo, timeData_t *timedata)
{

	timedata->time_date[SEC] = dec2bcd( timeinfo->tm_sec );
	timedata->time_date[MINUTE] = dec2bcd( timeinfo->tm_min );
	timedata->time_date[HOUR] = dec2bcd( timeinfo->tm_hour );
	timedata->time_date[DAY] = dec2bcd( timeinfo->tm_mday );
	timedata->time_date[MONTH] = dec2bcd( timeinfo->tm_mon + 1 );
	timedata->time_date[YEAR] = dec2bcd( timeinfo->tm_year - 100 );

	switch(timeinfo->tm_wday)
		{
			case 0 : timedata->time_date[WEEK] = SUNDAY;
				break;

			case 1 : timedata->time_date[WEEK] = MONDAY;
				break;

			case 2 : timedata->time_date[WEEK] = TUESDAY;
				break;

			case 3 : timedata->time_date[WEEK] = WEDNESDAY;
				break;

			case 4 : timedata->time_date[WEEK] = THURSDAY;
				break;

			case 5 : timedata->time_date[WEEK] = FRIDAY;
				break;

			case 6 : timedata->time_date[WEEK] = SATURDAY;
				break;

			default : timedata->time_date[WEEK]  = 0xff;  // invalid
		}

	return 1;
}

void update_display_time(void)
{


		timeData_t timeData;


		timeinfo = localtime( &now );

		RX8804_time_from_tm(timeinfo, &timeData);

		printf("set RTC to (BDC):%x,%x,%x,%x,%x,%x,%x\n",timeData.time_date[0],
				timeData.time_date[1],timeData.time_date[2],timeData.time_date[3],
				timeData.time_date[4],timeData.time_date[5],timeData.time_date[6]);

		// indicate new data
		timeData.setTime =  true;

}

#endif


// MPPT_DATA_RDY
// POND_DATA_RDY
// WEATHER_DATA_RDY
void loop_task(void *pvParameter)
{
	uint32_t incomingStatus = 0;

	pondData_t pond_data;
	MPPTdata_t mppt_data;
	weatherData_t weather_data;

	char tmp_str[17];

	uint16_t i = 0;
	uint16_t HBcount = 0;

	printf("Loop Task Started\n");
	while(1)
	{
		// check for incoming messages
		incomingStatus = getDataReadyStatus( );
		if( incomingStatus != NO_DATA_RDY )
		{
			// check for calibration  data update
			if( ( incomingStatus & MPPT_DATA_RDY ) == MPPT_DATA_RDY )
			{
				updateMPPT( &mppt_data );

			}

			if( ( incomingStatus & WEATHER_DATA_RDY )  == WEATHER_DATA_RDY )
			{
				updateWeather( &weather_data );
				if( weather_data.location_id == WEST_SIDE )
				{
					sprintf(tmp_str, "H: %2.1f ",weather_data.humidity );
					update_display(tmp_str, 1, 9);
					sprintf(tmp_str, "AT:%2.1f ",weather_data.temperature);
					update_display(tmp_str, 0, 9);
				}
			}

			// check for pond data update
			if( ( incomingStatus & POND_DATA_RDY ) == POND_DATA_RDY )
			{
				updatePond( &pond_data );
				// display resutls
				sprintf(tmp_str, "pH:%2.1f ",pond_data.pH);
				update_display(tmp_str, 1, 1);
				pond_data.water_temperature = ( pond_data.water_temperature * 1.8 ) + 32;
				sprintf(tmp_str, "WT:%2.1f ",pond_data.water_temperature);
				update_display(tmp_str, 0, 1);
				sprintf(tmp_str, "Light:%u[%d]:%lu",pond_data.light_level, pond_data.hour, pond_data.hourly_light_accum);
				//update_display(tmp_str, 3, 0);

			}
		}

		if( HBcount++ >= 3 )
		{
			HBcount = 0;
			update_display(heartbeat[i++], 0, 0);
			if( i == 2 ) i = 0;
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}

}
