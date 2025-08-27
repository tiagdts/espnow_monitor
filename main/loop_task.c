/*
 * loop_task.c
 *
 *  Created on: Oct 21, 2024
 *      Author: tiagd
 */
#include "loop_task.h"

extern char LCD_degStr[];
extern char LCD_pctStr[];
extern char LCD_pHstr[];

//static time_t now = 0;
//static struct tm *timeinfo;

static const char *windHeading[] = {
										"N",	// 0
										"NbE",  // 1
										"NNE",	// 2
										"NEbN",	// 3
										"NE",	// 4
										"NEbE", // 5
										"ENE",  // 6
										"EbN",  // 7
										"E",    // 8
										"EbS",  // 9
										"ESE",  // 10
										"SEbE", // 11
										"SE",   // 12
										"SEbS", // 13
										"SSE",  // 14
										"SbE",  // 15
										"S",    // 16
										"SbW",  // 17
										"SSW",  // 18
										"SWbS", // 19
										"SW",   // 20
										"SWbW", // 21
										"WSW",  // 22
										"WbS",  // 23
										"W",    // 24
										"WbN",  // 25
										"NWbW", // 26
										"NW",   // 27
										"NWbN", // 28
										"NNW",  // 29
										"NbW",  // 30
										"N",    // 31
										"---"	// 32  no reading
};

static char heartbeat[2][3] = {
								"*",
								" "
							};

extern SemaphoreHandle_t xSemaphore_LCD;

void getCompassRose(double heading, char *marker )
{
	uint32_t index;

	index = heading/11.25;
	strcpy(marker, windHeading[index]);
}


void format_wind(float speed, double direction, char *outStr )
{
	double compass;
	char compass_rose_marker[10];

	// get integer of direction
	modf(direction, &compass);
	getCompassRose(compass, compass_rose_marker);

	sprintf(outStr,"%s-%1.0fmph",compass_rose_marker, speed );
}

uint8_t update_display(char* message, uint8_t line, uint8_t col )
{
	uint8_t len;

	if( xSemaphoreTake( xSemaphore_LCD, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		LCD_setCursor(col, line);
		vTaskDelay(25 / portTICK_PERIOD_MS);
		LCD_writeStr(message);
		// give up control of LCD
		xSemaphoreGive( xSemaphore_LCD );
	}

	len = strlen( message )+1;
	len += 1;
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
	rainData_t rain_data;
	weatherData_t weather_data;
	ductData_t duct_data;

	char tmp_str[17];
	char air_blank[22] = "---------------------";
	char water_blank[18] = "-----------------";

	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t HBcount = 0;
	time_t now;
	time_t line_zero_time[ DATA_COUNT ] = {0,0};

	printf("Loop Task Started\n");
	while(1)
	{
		// check for incoming messages6
		incomingStatus = getDataReadyStatus( );
		if( incomingStatus != NO_DATA_RDY )
		{
#define SCROLL_DATA
#ifdef SCROLL_DATA

			if( ( incomingStatus & DUCT_DATA_RDY )  == DUCT_DATA_RDY )
			{
				updateDuct(&duct_data );
//				if( duct_data.location_id == FRONT_YARD )
//				{
					time(&now);
					time( &line_zero_time[AIR_DATA] );
					sprintf(tmp_str, "H: %2.1f%s",duct_data.air_humidity, LCD_pctStr );
					update_display(tmp_str, 0, 29);
					sprintf(tmp_str, "AT:%2.1f%s",duct_data.air_temperature, LCD_degStr);
					update_display(tmp_str, 0, 18);

					sprintf(tmp_str, "SOC:%2.1lf%s",duct_data.batt_soc, LCD_pctStr );
					LCD_add_scroll_data(DUCT_DATA, duct_data.location_id, TEXT_DATA, now, tmp_str);

					sprintf(tmp_str, "BAT:%2.2lf",duct_data.batt_volts );
					LCD_add_scroll_data(DUCT_DATA, duct_data.location_id, VOLT_DATA, now, tmp_str);

					sprintf(tmp_str, "BARO:%2.2f",duct_data.air_pressure );
					LCD_add_scroll_data(DUCT_DATA, duct_data.location_id, BAROMETRIC_PRESSURE_DATA, now, tmp_str);

					LCD_buildScrollString( );


//				}
			}

			// check for MPPT  data update
			if( ( incomingStatus & MPPT_DATA_RDY ) == MPPT_DATA_RDY )
			{
				updateMPPT( &mppt_data );
				//if( mppt_data.location_id == POND )
				//{
					time(&now);
					sprintf(tmp_str, "BAT:%2.2f",mppt_data.peak_charge_volts);
					LCD_add_scroll_data(MPPT_DATA, mppt_data.location_id, VOLT_DATA, now, tmp_str);
					//update_display(tmp_str, 0, 18);
					sprintf(tmp_str, "CHG:%5u",mppt_data.charge );
					LCD_add_scroll_data(MPPT_DATA, mppt_data.location_id, CHARGE_DATA, now, tmp_str);
					LCD_buildScrollString( );
					//update_display(tmp_str, 1, 18);
					log_data( &mppt_data, MPPT_DATA );
				//}
			}

			if( ( incomingStatus & RAIN_DATA_RDY ) == RAIN_DATA_RDY )
			{
				updateRain( &rain_data );
				time(&now);
				if( rain_data.accumulation_1hour != 0 )
				{
					sprintf(tmp_str, "1 hr Accum:%2.2f",rain_data.accumulation_1hour);
					LCD_add_scroll_data(RAIN_DATA, rain_data.location_id, TEXT_DATA, now, tmp_str);
					LCD_buildScrollString( );
				}
				else LCD_delete_scroll_data(RAIN_DATA, rain_data.location_id, TEXT_DATA);

				if( rain_data.accumulation_24hour != 0 )
				{
					sprintf(tmp_str, "24 hr Accum:%2.2f",rain_data.accumulation_24hour);
					LCD_add_scroll_data(RAIN_DATA, rain_data.location_id, TIME_DATA, now, tmp_str);
					LCD_buildScrollString( );
				}
				else LCD_delete_scroll_data(RAIN_DATA, rain_data.location_id, TIME_DATA);
			}
#endif
			if( ( incomingStatus & WEATHER_DATA_RDY )  == WEATHER_DATA_RDY )
			{
				updateWeather( &weather_data );
				if( weather_data.location_id == WEST_SIDE )
				{
					sprintf(tmp_str, "H: %2.1f%s",weather_data.humidity, LCD_pctStr );
					update_display(tmp_str, 0, 29);
					sprintf(tmp_str, "AT:%2.1f%s",weather_data.temperature, LCD_degStr);
					update_display(tmp_str, 0, 18);
					log_data( &weather_data, WEATHER_DATA );
				}
#ifdef SCROLL_DATA
				else if( weather_data.location_id == ROOF )
				{
					time(&now);
#ifdef OLD_FORMAT
					sprintf(tmp_str, "Wind:%2.0f",weather_data.wind_velocity);
					LCD_add_scroll_data(WEATHER_DATA, ROOF, VELOCITY_DATA, now, tmp_str);
					//update_display(tmp_str, 0, 29);
					sprintf(tmp_str, "Dir:%3.0f",weather_data.wind_direction);
#endif
					format_wind(weather_data.wind_velocity, (double)(weather_data.wind_direction), tmp_str );
					LCD_add_scroll_data(WEATHER_DATA, ROOF, TEXT_DATA, now, tmp_str);
					LCD_buildScrollString( );
					//update_display(tmp_str, 1, 29);
					log_data( &weather_data, WEATHER_DATA );
				}
#endif
			}

			// check for pond data update
			if( ( incomingStatus & POND_DATA_RDY ) == POND_DATA_RDY )
			{
				time( &line_zero_time[WATER_DATA] );
				updatePond( &pond_data );
				// display resutls
				sprintf(tmp_str, "%2.1f%s ",pond_data.pH,LCD_pHstr);
				update_display(tmp_str, 0, 10);
				pond_data.water_temperature = ( pond_data.water_temperature * 1.8 ) + 32;
				sprintf(tmp_str, "WT:%2.1f%s",pond_data.water_temperature,LCD_degStr);
				update_display(tmp_str, 0, 1);
				log_data( &pond_data, POND_DATA );

			}
		}
		else
		{
			// check for old data
			time(&now);
			for(j=0;j<DATA_COUNT;j++)
			{
				if( (now - line_zero_time[j] ) > TIME_OUT )
				{
					if(j == AIR_DATA )
					{
						update_display(air_blank, 0, 18);
					}
					if(j == WATER_DATA )
					{
						update_display(water_blank, 0, 1);
					}
				}
			}

		}

		if( HBcount++ >= 3 )
		{
			HBcount = 0;
			update_display(heartbeat[i++], 0, 0);
			if( i == 2 ) i = 0;
		}
		vTaskDelay(50/ portTICK_PERIOD_MS);
	}

}
