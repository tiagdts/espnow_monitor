/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "espnow_.h"
#include "esp_now.h"
#include "ds3231.h"

//#define TEST_SYSTEM_TIME_UPDATE

#define ESP_WIFI_SSID      "Wow9939"
#define ESP_WIFI_PASS      "8437955620"
#define MAX_STA_CONN       10

#define WIFI_SENSOR

//#define TEST

// define the number of types of data to sent
#define NUMBER_OF_TYPES 	2
//#define NUMBER_OF_TYPES 	4

#define DATA_STATUS			0
#define DATA_TYPE			1
#define TIMES_SENT			2
#define READY_TO_SLEEP		3

// define times to repeat sending data
#define SEND_COUNT 2

//static uint8_t send_count = 0;
static bool readyToSleep = false;

static uint32_t dataReadyStatus = 0;
static uint32_t dataNewStatus = 0;

static const char *TAG = "espnow";

static xQueueHandle s_espnow_queue;

// semaphores used between non-interrupt tasks
SemaphoreHandle_t xSemaphore_data_access = NULL;


// indicate types of data to be sent for this sensor
// this one is setup for weather data with solar reporting
//#define WEATHER_WITH_SOLAR
#ifdef WEATHER_WITH_SOLAR

//static uint32_t DataTypesToSend[NUMBER_OF_TYPES][4] = { // Data Ready, Data Type, sent count, ready to sleep
//														{ WEATHER_DATA_RDY, WEATHER_DATA, 0, 0 } ,
//														{ MPPT_DATA_RDY, MPPT_DATA, 0, 0},
//														 // this one is initialized in active, not sent every sleep cycle
//													  };
static uint32_t DataTypesToSend[NUMBER_OF_TYPES][4] = { // Data Ready, Data Type, sent count, ready to sleep
														{ WEATHER_DATA_RDY, WEATHER_DATA, 0, 0 } ,
														{ MPPT_DATA_RDY, MPPT_DATA, 0, 0 },
														{ PHONE_DATA_RDY, PHONE_DATA, 0, 0 },
														{ RAIN_DATA_RDY, RAIN_DATA, 0, 0 }
													  };
#endif

#define TIME_KEEPER
#ifdef TIME_KEEPER
static uint32_t DataTypesToSend[NUMBER_OF_TYPES][4] = { // Data Ready, Data Type, sent count, ready to sleep
														{ SYSTEM_TIME_DATA_RDY, SYSTEM_TIME_DATA, 0, 0 },
														{ WEATHER_CAL_DATA_RDY, WEATHER_CAL_DATA, 0, 0}
													  };
#endif

//static uint16_t current_type = 0;

//static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#ifdef DISPLAY_ONLY
static uint8_t s_unicast_mac[ESP_NOW_ETH_ALEN] = { 0x24, 0x0a, 0xc4, 0x1c, 0x9d, 0x41 }; // Display
#else
static uint8_t s_unicast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // everyone
#endif

static uint16_t loc_seq_num = 0;

// Weather data
weatherData_t loc_weatherData;

// Rain data
rainData_t loc_rainData;

// Pump data
pumpData_t loc_pumpData;

// Phone data
phoneData_t loc_phoneData;

// HVAC data
HVACdata_t loc_HVACdata;

// room data
roomData_t loc_roomData;

// time data
timeData_t loc_timeData;

// system time data
systemTimeData_t loc_systemTimeData;

// MPPT data
MPPTdata_t loc_MPPTdata;

// weather sensor calibration data
weatherCalibrationData_t loc_weatherCalData;

// No data
NoData_t loc_NoData;

void clrDataTypesToSendAll(void)
{
	uint32_t i;

	for(i=0;i<NUMBER_OF_TYPES;i++)
	{
		DataTypesToSend[i][ TIMES_SENT ] = 0;
		DataTypesToSend[i][ READY_TO_SLEEP ] = 0;
	}
}

void clrDataTypesToSendIndividual(uint32_t i)
{
	DataTypesToSend[i][ TIMES_SENT ] = 0;
	DataTypesToSend[i][ READY_TO_SLEEP ] = 0;

}

void setDataTypesToSendIndividual(uint32_t i)
{
	DataTypesToSend[i][ TIMES_SENT ] = SEND_COUNT;
	DataTypesToSend[i][ READY_TO_SLEEP ] = 1;

}

bool getReadyToSleep(void)
{
	return readyToSleep;
}

void clrReadyToSleep(void)
{
	readyToSleep = false;
}

void clearDataReadyStatus( uint32_t clearbits)
{
	uint32_t tmp = ~clearbits;
	dataReadyStatus = dataReadyStatus & tmp;
}

uint32_t getDataReadyStatus( void )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		uint32_t tmp = dataReadyStatus;
		xSemaphoreGive( xSemaphore_data_access );

		return tmp;
	}

	return NO_DATA_RDY;
}

// check for new data available and times sent
static uint32_t check_data_status( void )
{
	uint32_t i;

	for(i=0;i<NUMBER_OF_TYPES;i++)
	{
		// check for new data
		if( dataNewStatus & DataTypesToSend[i][DATA_STATUS] )
		{
			// increment send count
			DataTypesToSend[i][TIMES_SENT]++;

			if( DataTypesToSend[i][TIMES_SENT] >= SEND_COUNT )
			{
				// clear new data status
				dataNewStatus = dataNewStatus & ~DataTypesToSend[i][DATA_STATUS];

				// indicate this sensor is finished
				DataTypesToSend[i][READY_TO_SLEEP] = 1;
			}
			return DataTypesToSend[i][DATA_TYPE];

		}
	}

	return NO_DATA;
}

// check for new data available and times sent
static uint32_t check_data_status_non_sleep( void )
{
	uint32_t i;

	for(i=0;i<NUMBER_OF_TYPES;i++)
	{
		// check for new data
		if( dataNewStatus & DataTypesToSend[i][DATA_STATUS] )
		{
			// clear new data status
			dataNewStatus = dataNewStatus & ~DataTypesToSend[i][DATA_STATUS];
			return DataTypesToSend[i][DATA_TYPE];
		}
	}

	return NO_DATA;
}

static bool check_sleep_status(void)
{
	uint32_t tmp_sleepStatus = 1;
	uint32_t i;

	for(i=0;i<NUMBER_OF_TYPES;i++)
	{
		// check to see if any data has been received
		if(DataTypesToSend[i][ TIMES_SENT ] == 0 ) return 0; // not ready to sleep, data has not been received
		tmp_sleepStatus = tmp_sleepStatus && DataTypesToSend[i][READY_TO_SLEEP];
	}
	if(tmp_sleepStatus) return true;
	else return false;

}

static bool check_sleep_status2(void)
{

	DataTypesToSend[0][ TIMES_SENT ]++;
	if(DataTypesToSend[0][ TIMES_SENT ] == SEND_COUNT )
	{
		DataTypesToSend[0][ TIMES_SENT ] = 0;
		return true;
	}
	else return 0; // not ready to sleep

}

static void initDataStructures(void)
{
	memset( &loc_weatherData, 0, sizeof( loc_weatherData ) );
	memset( &loc_rainData, 0, sizeof( loc_rainData ) );
	memset( &loc_pumpData, 0, sizeof( loc_pumpData ) );
	memset( &loc_phoneData, 0, sizeof( loc_phoneData ) );
	memset( &loc_HVACdata, 0, sizeof( loc_HVACdata ) );
	memset( &loc_roomData, 0, sizeof( loc_roomData ) );
	memset( &loc_timeData, 0, sizeof( loc_timeData ) );
	memset( &loc_systemTimeData, 0, sizeof( loc_systemTimeData ) );
	memset( &loc_MPPTdata, 0, sizeof( loc_MPPTdata ) );
	memset( &loc_NoData, 0, sizeof( loc_NoData ) );
	memset( &loc_weatherCalData, 0, sizeof( loc_weatherCalData ) );
	// Initialize loc_NoData, it never gets updated
	strcpy( (char*)(loc_NoData.no_data), "NO DATA");
}


static void espnow_deinit(espnow_send_param_t *send_param);

static void printWeatherData(void)
{
	printf("Weather Data: %d, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n\r", loc_weatherData.location_id,
			loc_weatherData.baro_pressure, loc_weatherData.humidity, loc_weatherData.temperature,
			loc_weatherData.wind_direction, loc_weatherData.wind_velocity);
}

int16_t updateWeather( weatherData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->baro_pressure = loc_weatherData.baro_pressure;
		data->humidity = loc_weatherData.humidity;
		data->location_id = loc_weatherData.location_id;
		data->temperature = loc_weatherData.temperature;
		data->wind_direction = loc_weatherData.wind_direction;
		data->wind_velocity = loc_weatherData.wind_velocity;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~WEATHER_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

int16_t updateWeatherloc( weatherData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_weatherData.baro_pressure = data->baro_pressure;
		loc_weatherData.humidity = data->humidity;
		loc_weatherData.location_id = data->location_id;
		loc_weatherData.temperature = data->temperature;
		loc_weatherData.wind_direction = data->wind_direction;
		loc_weatherData.wind_velocity = data->wind_velocity;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | WEATHER_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

static void printRainData(void)
{
	printf("Rain Data: %d, %2.2f, %2.2f, %3.2f\n\r", loc_rainData.location_id,
			loc_rainData.accumulation_1hour, loc_rainData.accumulation_24hour, loc_rainData.rate);
}

int16_t updateRain( rainData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->accumulation_1hour = loc_rainData.accumulation_1hour;
		data->accumulation_24hour = loc_rainData.accumulation_24hour;
		data->rate = loc_rainData.rate;
		data->location_id = loc_rainData.location_id;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~RAIN_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

int16_t updateRainloc( rainData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_rainData.accumulation_1hour = data->accumulation_1hour;
		loc_rainData.accumulation_24hour = data->accumulation_24hour;
		loc_rainData.rate = data->rate;
		loc_rainData.location_id = data->location_id;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | RAIN_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

static void printPumpData(void)
{
	printf("Pump Data: %d, %3.2f, %3.2f, %3.2f, %3.2f\n\r", loc_pumpData.location_id,
			loc_pumpData.output_pressure, loc_pumpData.output_rate, loc_pumpData.output_volume, loc_pumpData.pump_temperature);
}

int16_t  updatePump( pumpData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->location_id = loc_pumpData.location_id;
		data->output_pressure = loc_pumpData.output_pressure;
		data->output_rate = loc_pumpData.output_rate;
		data->output_volume = loc_pumpData.output_volume;
		data->pump_temperature = loc_pumpData.pump_temperature;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~PUMP_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

int16_t  updatePumploc( pumpData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_pumpData.location_id = data->location_id;
		loc_pumpData.output_pressure = data->output_pressure;
		loc_pumpData.output_rate = data->output_rate;
		loc_pumpData.output_volume = data->output_volume;
		loc_pumpData.pump_temperature = data->pump_temperature;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | PUMP_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

static void printPhoneData(void)
{
	printf("Phone Data: %d, %s, %s, %s, %s\n\r", loc_phoneData.location_id,
			loc_phoneData.date_str, loc_phoneData.Name_str, loc_phoneData.number_str, loc_phoneData.time_str);
}


int16_t  updatePhone( phoneData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{

		memcpy( data->date_str, loc_phoneData.date_str, sizeof( data->date_str ) );
		memcpy( data->Name_str, loc_phoneData.Name_str, sizeof( data->Name_str) );
		memcpy( data->number_str, loc_phoneData.number_str, sizeof( data->number_str ) );
		memcpy( data->time_str, loc_phoneData.time_str, sizeof( data->time_str ) );
		data->location_id = loc_phoneData.location_id;
		data->data_valid = loc_phoneData.data_valid;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~PHONE_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

int16_t  updatePhoneloc( phoneData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{

		memcpy(  loc_phoneData.date_str, data->date_str, sizeof( data->date_str ) );
		memcpy(  loc_phoneData.Name_str, data->Name_str,sizeof( data->Name_str) );
		memcpy( loc_phoneData.number_str, data->number_str, sizeof( data->number_str ) );
		memcpy( loc_phoneData.time_str, data->time_str, sizeof( data->time_str ) );
		loc_phoneData.location_id = data->location_id;
		loc_phoneData.data_valid = data->data_valid;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | PHONE_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

static void printHVACdata(void)
{
	printf("HVAC Data: %d, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n\r", loc_HVACdata.location_id,
			loc_HVACdata.air_flow, loc_HVACdata.ambient_temp, loc_HVACdata.compressor_current, loc_HVACdata.fan_current,
			loc_HVACdata.humidity, loc_HVACdata.inlet_temp, loc_HVACdata.outlet_temp );
}


int16_t  updateHVAC( HVACdata_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->air_flow = loc_HVACdata.air_flow;
		data->ambient_temp = loc_HVACdata.ambient_temp;
		data->compressor_current = loc_HVACdata.compressor_current;
		data->fan_current = loc_HVACdata.fan_current;
		data->humidity = loc_HVACdata.humidity;
		data->inlet_temp = loc_HVACdata.inlet_temp;
		data->location_id = loc_HVACdata.location_id;
		data->outlet_temp = loc_HVACdata.outlet_temp;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~HVAC_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

static void printRoomData(void)
{
	printf("Room Data: %d, %3.2f, %3.2f, %3.2f\n\r", loc_roomData.location_id,
			loc_roomData.air_quality, loc_roomData.humidity, loc_roomData.temperature);
}

int16_t  updateRoom( roomData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->air_quality = loc_roomData.air_quality;
		data->humidity = loc_roomData.humidity;
		data->location_id = loc_roomData.location_id;
		data->occupied = loc_roomData.occupied;
		data->temperature = loc_roomData.temperature;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~ROOM_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

int16_t  updateRoomloc( roomData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_roomData.air_quality = data->air_quality;
		loc_roomData.humidity = data->humidity;
		loc_roomData.location_id = data->location_id;
		loc_roomData.occupied = data->occupied;
		loc_roomData.temperature = data->temperature;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | ROOM_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}


static void printTimeData(void)
{
	printf("Time Data: %x, %x, %x, %x, %x, %x, %x\n\r", loc_timeData.time_date[HOUR], loc_timeData.time_date[MINUTE],
														loc_timeData.time_date[SEC],loc_timeData.time_date[MONTH],
														loc_timeData.time_date[DAY],loc_timeData.time_date[YEAR],
														loc_timeData.time_date[WEEK] );

}

static void printSystemTimeData(void)
{

	printf("System time: %s, %ld sec, %ld usec\n", loc_systemTimeData.description, loc_systemTimeData.t.tv_sec, loc_systemTimeData.t.tv_usec);
}

int16_t  updateTime( timeData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->setTime = loc_timeData.setTime;
		data->time_date[HOUR] = loc_timeData.time_date[HOUR];
		data->time_date[MINUTE] = loc_timeData.time_date[MINUTE];
		data->time_date[SEC] = loc_timeData.time_date[SEC];
		data->time_date[MONTH] = loc_timeData.time_date[MONTH];
		data->time_date[DAY] = loc_timeData.time_date[DAY];
		data->time_date[YEAR] = loc_timeData.time_date[YEAR];
		data->time_date[WEEK] = loc_timeData.time_date[WEEK];

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~TIME_DATA_RDY;

		// clear "setTime" variable in local data
		//loc_timeData.setTime = false;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}


int16_t updateSystemTime( systemTimeData_t *data)
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
		{
			memcpy( data->description, loc_systemTimeData.description, sizeof( data->description) );
			data->t.tv_sec = loc_systemTimeData.t.tv_sec;
			data->t.tv_usec = loc_systemTimeData.t.tv_usec;

			// clear status bit
			dataReadyStatus = dataReadyStatus & ~SYSTEM_TIME_DATA_RDY;

			xSemaphoreGive( xSemaphore_data_access );

			return DATA_READ;
		}

		return DATA_READ_TIMEOUT;
}


int16_t  updateTimeloc( timeData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_timeData.setTime = data->setTime;
		loc_timeData.time_date[HOUR] = data->time_date[HOUR];
		loc_timeData.time_date[MINUTE] = data->time_date[MINUTE];
		loc_timeData.time_date[SEC] = data->time_date[SEC];
		loc_timeData.time_date[MONTH] = data->time_date[MONTH];
		loc_timeData.time_date[DAY] = data->time_date[DAY];
		loc_timeData.time_date[YEAR] = data->time_date[YEAR];
		loc_timeData.time_date[WEEK] =  data->time_date[WEEK];

		// indicate data new since last send
		dataNewStatus = dataNewStatus | TIME_DATA_RDY;

		// clear "setTime" flag in local data
		//loc_timeData.setTime = false;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}


int16_t  updateSystemTimeloc( systemTimeData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		memcpy( loc_systemTimeData.description, data->description, sizeof( data->description) );
		loc_systemTimeData.t.tv_sec = data->t.tv_sec;
		loc_systemTimeData.t.tv_usec = data->t.tv_usec;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | SYSTEM_TIME_DATA_RDY;
		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

void setMPPTdataOld(void)
{
	loc_MPPTdata.new_data = false;
}

int16_t updateMPPTloc( MPPTdata_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_MPPTdata.new_data = data->new_data;
		loc_MPPTdata.charge = data->charge;
		loc_MPPTdata.location_id = data->location_id;
		loc_MPPTdata.peak_charge_current = data->peak_charge_current;
		loc_MPPTdata.peak_charge_volts = data->peak_charge_volts;
		loc_MPPTdata.peak_solar_volts = data->peak_solar_volts;
		loc_MPPTdata.peak_watts = data->peak_watts;
		loc_MPPTdata.wiper = data->wiper;
		loc_MPPTdata.charger_temp =  data->charger_temp;
		loc_MPPTdata.time = data->time;

		// indicate data new since last send
		dataNewStatus = dataNewStatus | MPPT_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

static void printMPPTdata(void)
{
	printf("MPPT Data: %ld, %u, %d, %u, ,%u, %3.2f, %3.2f, %3.2f, %3.2f, %3.1f\n\r", loc_MPPTdata.time, loc_MPPTdata.new_data, loc_MPPTdata.wiper, loc_MPPTdata.location_id,
			loc_MPPTdata.charge, loc_MPPTdata.peak_charge_current, loc_MPPTdata.peak_charge_volts,
			loc_MPPTdata.peak_watts, loc_MPPTdata.peak_solar_volts, loc_MPPTdata.charger_temp);
}

int16_t updateMPPT( MPPTdata_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->new_data = loc_MPPTdata.new_data;
		data->charge = loc_MPPTdata.charge;
		data->location_id = loc_MPPTdata.location_id;
		data->peak_charge_current = loc_MPPTdata.peak_charge_current;
		data->peak_charge_volts = loc_MPPTdata.peak_charge_volts;
		data->peak_solar_volts = loc_MPPTdata.peak_solar_volts;
		data->peak_watts = loc_MPPTdata.peak_watts;
		data->wiper = loc_MPPTdata.wiper;
		data->charger_temp = loc_MPPTdata.charger_temp;
		data->time = loc_MPPTdata.time;

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~WEATHER_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

int16_t updateNoData( NoData_t *data )
{

	 strcpy((char*)(data->no_data), (char*)(loc_NoData.no_data));

	return DATA_READ;

}

static void printNodata(void)
{
	printf("No Data: %s\n\r", loc_NoData.no_data);
}


static void downloadWeather( weatherData_t *data )
{
	loc_weatherData.baro_pressure = data->baro_pressure;
	loc_weatherData.humidity = data->humidity;
	loc_weatherData.location_id = data->location_id;
	loc_weatherData.temperature = data->temperature;
	loc_weatherData.wind_direction = data->wind_direction;
	loc_weatherData.wind_velocity = data->wind_velocity;
}

static void downloadRain( rainData_t *data )
{
	loc_rainData.accumulation_1hour = data->accumulation_1hour;
	loc_rainData.accumulation_24hour = data->accumulation_24hour;
	loc_rainData.rate = data->rate;
	loc_rainData.location_id = data->location_id;
}

static void downloadPump( pumpData_t *data )
{
	loc_pumpData.location_id = data->location_id;
	loc_pumpData.output_pressure = data->output_pressure;
	loc_pumpData.output_rate = data->output_rate;
	loc_pumpData.output_volume = data->output_volume;
	loc_pumpData.pump_temperature = data->pump_temperature;
}

static void downloadPhone( phoneData_t *data )
{
	strcpy( loc_phoneData.date_str, data->date_str );
	strcpy( loc_phoneData.Name_str, data->Name_str );
	strcpy( loc_phoneData.number_str, data->number_str );
	strcpy( loc_phoneData.time_str, data->time_str );

	loc_phoneData.location_id = data->location_id;
	loc_phoneData.data_valid = data->data_valid;

}

static void downloadHVAC( HVACdata_t *data )
{
	loc_HVACdata.air_flow = data->air_flow;
	loc_HVACdata.ambient_temp = data->ambient_temp;
	loc_HVACdata.compressor_current = data->compressor_current;
	loc_HVACdata.fan_current = data->fan_current;
	loc_HVACdata.humidity = data->humidity;
	loc_HVACdata.inlet_temp = data->inlet_temp;
	loc_HVACdata.location_id = data->location_id;
	loc_HVACdata.outlet_temp = data->outlet_temp;
}

static void downloadRoom( roomData_t *data )
{
	loc_roomData.air_quality = data->air_quality;
	loc_roomData.humidity = data->humidity;
	loc_roomData.location_id = data->location_id;
	loc_roomData.occupied = data->occupied;
	loc_roomData.temperature = data->temperature;
}

static void downloadTime( timeData_t *data )
{
	loc_timeData.setTime = data->setTime;
	loc_timeData.time_date[HOUR] = data->time_date[HOUR];
	loc_timeData.time_date[MINUTE] = data->time_date[MINUTE];
	loc_timeData.time_date[SEC] = data->time_date[SEC];
	loc_timeData.time_date[DAY] = data->time_date[DAY];
	loc_timeData.time_date[YEAR] = data->time_date[YEAR];
	loc_timeData.time_date[WEEK] = data->time_date[WEEK];

}


static void downloadSystemTime( systemTimeData_t *data )
{
	memcpy( loc_systemTimeData.description, data->description, sizeof( data->description) );
	loc_systemTimeData.t.tv_sec = data->t.tv_sec;
	loc_systemTimeData.t.tv_usec = data->t.tv_usec;

}

static void downloadMPPT( MPPTdata_t *data )
{
	loc_MPPTdata.charge = data->charge;
	loc_MPPTdata.location_id = data->location_id;
	loc_MPPTdata.peak_charge_current = data->peak_charge_current;
	loc_MPPTdata.peak_charge_volts = data->peak_charge_volts;
	loc_MPPTdata.peak_solar_volts = data->peak_solar_volts;
	loc_MPPTdata.peak_watts = data->peak_watts;
	loc_MPPTdata.wiper = data->wiper;
	loc_MPPTdata.time = data->time;
	loc_MPPTdata.charger_temp = data->charger_temp;

}

static void downloadWeatherCal( weatherCalibrationData_t *data )
{

	loc_weatherCalData.location_id = data->location_id;
	int32_t i;

	for(i=0;i<5;i++)
		loc_weatherCalData.calibration_data[i] = data->calibration_data[i];

}

int16_t updateWeatherCalLoc( weatherCalibrationData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		loc_weatherCalData.location_id = data->location_id;
		int32_t i;

		for(i=0;i<5;i++)
			loc_weatherCalData.calibration_data[i] = data->calibration_data[i];

		// indicate data new since last send
		dataNewStatus = dataNewStatus | WEATHER_CAL_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;

}

int16_t updateWeatherCal( weatherCalibrationData_t *data )
{
	if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		data->location_id = loc_weatherCalData.location_id;
		int32_t i;

		for(i=0;i<5;i++)
			data->calibration_data[i] = loc_weatherCalData.calibration_data[i];

		// clear status bit
		dataReadyStatus = dataReadyStatus & ~WEATHER_CAL_DATA_RDY;

		xSemaphoreGive( xSemaphore_data_access );

		return DATA_READ;
	}

	return DATA_READ_TIMEOUT;
}

static void printWeatherCalData(void)
{
	printf("Weather Cal Data: %u, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n\r", loc_weatherCalData.location_id,
			loc_weatherCalData.calibration_data[0], loc_weatherCalData.calibration_data[1],
			loc_weatherCalData.calibration_data[2], loc_weatherCalData.calibration_data[3],
			loc_weatherCalData.calibration_data[4]);
}

/* WiFi should start before using ESPNOW */
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK( esp_event_loop_delete_default() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );

    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}


/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint16_t *seq, uint8_t *payload )
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if ( data_len < ESPNOW_SEND_LEN ) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        payload = NULL;
        return -1;
    }

    //*state = buf->state;
    *seq = buf->seq_num;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc)
    {
    	switch( buf->payload_type )
    	{
    		case WEATHER_DATA :
    				 memcpy(payload, buf->payload, sizeof( weatherData_t ) );
    			break;

    		case PUMP_DATA :
    			   memcpy(payload, buf->payload, sizeof( pumpData_t ) );
    			break;

    		case PHONE_DATA :
    			    memcpy(payload, buf->payload, sizeof( phoneData_t ) );
    			break;

    		case HVAC_DATA :
    				memcpy(payload, buf->payload, sizeof( HVACdata_t ) );
    			break;

    		case ROOM_DATA :
    			    memcpy(payload, buf->payload, sizeof( roomData_t ) );
    			break;

    		case TIME_DATA :
    			    memcpy(payload, buf->payload, sizeof( timeData_t ) );
    			break;

    		case MPPT_DATA :
    			    memcpy(payload, buf->payload, sizeof( MPPTdata_t ) );
    			break;

    		case SYSTEM_TIME_DATA :
    		    	memcpy(payload, buf->payload, sizeof( systemTimeData_t ) );
    		    break;

    		case RAIN_DATA :
		    		memcpy(payload, buf->payload, sizeof( rainData_t ) );
		    	break;

    		case WEATHER_CAL_DATA :
		    		memcpy(payload, buf->payload, sizeof( weatherCalibrationData_t ) );
		    	break;

    		default :
    			buf->payload_type =  NO_DATA;
    			//payload = NULL;
    	}

        return buf->payload_type;
    }

    return CRC_FAIL;
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t dataType )
{
	uint16_t espnow_data_t_size = 0;
    espnow_data_t *buf = (espnow_data_t *)(send_param->buffer);
    espnow_data_t_size = sizeof(espnow_data_t);
    assert( send_param->len >= espnow_data_t_size );

    // clear buffer memory
    memset(send_param->buffer, 0, sizeof(espnow_data_t) );

    buf->seq_num = loc_seq_num++;

    buf->payload_type = dataType;
    switch( dataType )
    {
		case WEATHER_DATA :
				// fill payload with current weather data
				updateWeather( (weatherData_t *) (&buf->payload ) );
				printWeatherData();
			break;

		case PUMP_DATA :
				// fill payload with current pump data
				updatePump( (pumpData_t *) (&buf->payload ) );
				printPumpData();
			break;

		case PHONE_DATA :
				// fill payload with current phone data
				updatePhone( (phoneData_t *) (&buf->payload ) );
				printPhoneData();
			break;

		case HVAC_DATA :
				// fill payload with current HVAC data
				updateHVAC( (HVACdata_t *) (&buf->payload ) );
				printHVACdata();
			break;

		case ROOM_DATA :
			// fill payload with current room data
			updateRoom( (roomData_t *) (&buf->payload ) );
			printRoomData();
			break;

		case TIME_DATA :
			// fill payload with current time data
			updateTime( (timeData_t *) (&buf->payload ) );
			printTimeData();
			break;

		case MPPT_DATA :
			// fill payload with current MPPT data
			updateMPPT( (MPPTdata_t *) (&buf->payload ) );
			printMPPTdata();
			// indicate data is old
			setMPPTdataOld();
			break;

		case SYSTEM_TIME_DATA :
			// fill payload with current system time data
			updateSystemTime( (systemTimeData_t *) (&buf->payload ) );
			printSystemTimeData();
			break;

		case RAIN_DATA :
			// fill payload with current rain data
			updateRain( (rainData_t *) (&buf->payload ) );
			printRainData();
			break;

		case WEATHER_CAL_DATA :
			// fill payload with current weather calibration data
			updateWeatherCal( (weatherCalibrationData_t *) (&buf->payload ) );
			printWeatherCalData();
			break;

		case NO_DATA :
			// fill payload with current no data
			updateNoData( (NoData_t *) (&buf->payload ) );
			printNodata();
			break;

			// unknown data type
		default : ;
    }



    buf->crc = 0;

    /* Fill all remaining bytes after the data with random values */
    //esp_fill_random(&buf->payload + sizeof( espnow_data_t ), send_param->len - sizeof(espnow_data_t));
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    //uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    //float recv_temperature = 0.0;
    uint8_t data_type = MPPT_DATA;
//    uint8_t update_data;

    uint8_t payload_pt[ESPNOW_SEND_LEN];

    int ret;

    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
#ifdef WIFI_SENSOR
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
#endif
    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                //is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

//#define SLEEP_MODE
#ifdef SLEEP_MODE

#ifdef TEST_SYSTEM_TIME_UPDATE
                // loop X number of times then set readyToSleep
                readyToSleep = check_sleep_status2();

#else
                // see if all the data has been sent before sleeping
                readyToSleep = check_sleep_status();
#endif
                // wait here until "readyToSleep" is cleared
				while(readyToSleep)
				{
					//vTaskDelay(send_param->delay/portTICK_RATE_MS);
					vTaskDelay(20/portTICK_RATE_MS);
					// old data will be sent after sleeping
				}
#endif
				/* Delay a while before sending the next data. */
				if (send_param->delay > 0) {
					vTaskDelay(send_param->delay/portTICK_RATE_MS);
				}

				// address data going to
                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
#ifdef SLEEP_MODE
                // See if there is data ready to send
                data_type = check_data_status( );
#else
               // NON-SLEEP FUNCTION
			   data_type = check_data_status_non_sleep( );
			   printf("Sending Data Type: %u\n", data_type );
#endif
                espnow_data_prepare( send_param, data_type );

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                // *payload_pt = NULL;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_seq, payload_pt );


                free(recv_cb->data);

				switch( ret )
				{
					case WEATHER_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadWeather( (weatherData_t *)( &payload_pt ) );
								dataReadyStatus = dataReadyStatus | WEATHER_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printWeatherData();
						break;

					case PUMP_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadPump( (pumpData_t *)( &payload_pt ) );
								dataReadyStatus = dataReadyStatus | PUMP_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printPumpData();
						break;

					case PHONE_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{

								downloadPhone( (phoneData_t *)( &payload_pt ) );

								// if it is not an empty record update status
								if( loc_phoneData.data_valid )
									dataReadyStatus = dataReadyStatus | PHONE_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printPhoneData();
						break;

					case HVAC_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadHVAC( (HVACdata_t *)( &payload_pt ) );
								dataReadyStatus = dataReadyStatus | HVAC_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printHVACdata();
						break;

					case ROOM_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadRoom( (roomData_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | ROOM_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printRoomData();
						break;

					case TIME_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadTime( (timeData_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | TIME_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printTimeData();
						break;

					case MPPT_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadMPPT( (MPPTdata_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | MPPT_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printMPPTdata();
						break;

					case SYSTEM_TIME_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadSystemTime( (systemTimeData_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | SYSTEM_TIME_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printSystemTimeData();
						break;

					case RAIN_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadRain( (rainData_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | RAIN_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printRainData();
						break;

					case WEATHER_CAL_DATA :
							if( xSemaphoreTake( xSemaphore_data_access, TASK_DATA_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
							{
								downloadWeatherCal( (weatherCalibrationData_t *)(&payload_pt) );
								dataReadyStatus = dataReadyStatus | WEATHER_CAL_DATA_RDY;

								xSemaphoreGive( xSemaphore_data_access );
							}
							printWeatherCalData();
						break;

				}

				break;

            }

            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;

    initDataStructures();

    // create mutex semaphores to be used for data access
    // 	between Tasks
    xSemaphore_data_access = xSemaphoreCreateMutex();

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_unicast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = ESPNOW_SEND_LEN;
    send_param->buffer = malloc( ESPNOW_SEND_LEN );
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_unicast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare( send_param, SYSTEM_TIME_DATA );

    xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_espnow_queue);
    esp_now_deinit();
}
