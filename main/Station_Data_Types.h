/*
 * Station_Data_Types.h
 *
 *  Created on: Jan 21, 2021
 *      Author: tiagd
 */

#ifndef MAIN_STATION_DATA_TYPES_H_
#define MAIN_STATION_DATA_TYPES_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include "stdbool.h"

#define CRC_FAIL				   -1
#define NO_DATA						0
#define TEMPERATURE_DATA			1
#define HUMIDITY_DATA				2
#define	BAROMETRIC_PRESSURE_DATA	3
#define VOLUME_DATA					4
#define	VELOCITY_DATA				5
#define	DISTANCE_DATA				6
#define	PRESSURE_DATA				7
#define	INTENSITY_DATA				8
#define	CALLER_ID_DATA				9
#define LOCATION_DATA				10
#define FLOW_RATE_DATA				11
#define	TEXT_DATA					12
#define TIME_DATA					13
#define VOLT_DATA					14
#define	AMP_DATA					15
#define MASS_DATA					16
#define AREA_DATA					17
#define ANGLE_DATA					18

#define WEATHER_DATA				19
#define PUMP_DATA					20
#define PHONE_DATA					21
#define HVAC_DATA					22
#define ROOM_DATA					23
#define CONFIGURATION_DATA			24
#define MPPT_DATA					25
#define SYSTEM_TIME_DATA			26
#define RAIN_DATA					27
#define WEATHER_CAL_DATA			28
#define POND_DATA					29
#define PH_CAL_DATA					30
#define BUTTON_DATA					32

	/* Locations inside */
#define LIVING_ROOM					1
#define KITCHEN						2
#define BEDROOM1					3
#define	BEDROOM2					4
#define SUNROOM						5
#define	UTILITY_ROOM				6
#define	HVAC_INSIDE_UNIT			7
#define SHOP_FRONT					8
#define SHOP_BACK					9

#define OUTSIDE						0x100

	/* locations outside */
#define FRONT_YARD					1 + OUTSIDE
#define BACK_YARD					2 + OUTSIDE
#define WEST_SIDE					3 + OUTSIDE
#define	EAST_SIDE					4 + OUTSIDE
#define GARAGE						5 + OUTSIDE
#define HVAC_OUTSIDE_UNIT			6 + OUTSIDE
#define ROOF						7 + OUTSIDE
#define POND						8 + OUTSIDE

// BUTTON types
#define BUTTON_TYPE_NULL	0
#define BUTTON_TYPE_PH_CAL	1

// BUTTON function
typedef enum  { IDLE_BTN, START_BTN, CHANGE_BTN } button_data_t;


typedef struct
{
	float temperature;
	float humidity;
	float baro_pressure;
	float wind_direction;
	float wind_velocity;
	uint16_t location_id;
	time_t time;
} weatherData_t;

typedef struct
{
	float accumulation_1hour;
	float accumulation_24hour;
	float rate; // one hour rate
	int hour;
	uint16_t location_id;
	time_t time;
} rainData_t;

typedef struct
{
	float pump_temperature;
	float output_volume;
	float output_pressure;
	float output_rate;
	uint16_t location_id;
	float pump_current;
	bool bypass_relay;
	bool button;
	time_t time;
} pumpData_t;


typedef struct
{
	bool data_valid;
	char date_str[6];
	char time_str[6];
	char number_str[16];
	char Name_str[20];
	uint16_t location_id;
} phoneData_t;

typedef struct
{
	float inlet_temp;
	float outlet_temp;
	float ambient_temp;
	float humidity;
	float compressor_current;
	float fan_current;
	float air_flow;
	uint16_t location_id;
} HVACdata_t;

typedef struct
{
	float temperature;
	float humidity;
	float air_quality;
	bool occupied;
	uint16_t location_id;
} roomData_t;

typedef struct
{
	bool setTime;
	uint8_t time_date[7];

} timeData_t;

typedef struct
{
	uint8_t description[40];
	struct timeval t;

} systemTimeData_t;

typedef struct
{
	uint8_t new_data;
	uint16_t charge;
	uint8_t wiper;
	float peak_watts;
	float peak_charge_volts;
	float peak_charge_current;
	float peak_solar_volts;
	float charger_temp;
	uint16_t location_id;
	time_t time;

} MPPTdata_t;

typedef struct
{
	uint8_t no_data[20];

} NoData_t;


typedef struct
{
	//char Header[5];
	float calibration_data[5];
	uint16_t location_id;

} weatherCalibrationData_t;

typedef struct
{
	//char Header[5];
	float air_temperature;
	float water_temperature;
	uint16_t light_level;
	float turbidity;
	float fluoresence;
	float pH;
	uint16_t location_id;
	time_t time;

} pondData_t;


typedef struct
{
	uint16_t state;
	float low_standard;
	float mid_standard;
	float high_standard;
	float pH_volts_low;
	float pH_volts_mid;
	float pH_volts_high;
	float coeff_exp;
	float coeff_slope;
	float coeff_intercept;
	float temp;
	float r;
	bool saved;
	time_t time;
} pHCalData_t;

typedef struct
{
	uint16_t location_id;
	uint32_t button_type;
	int16_t button_data;
	int16_t state;
	int16_t last_state;
	int16_t next_state;
	time_t time;
} buttonData_t;


#define NO_DATA_RDY					0x00000000
#define WEATHER_DATA_RDY			0x00000001
#define PUMP_DATA_RDY				0x00000002
#define HVAC_DATA_RDY				0x00000004
#define PHONE_DATA_RDY				0X00000008
#define ROOM_DATA_RDY				0x00000010
#define TIME_DATA_RDY				0x00000020
#define MPPT_DATA_RDY				0x00000040
#define SYSTEM_TIME_DATA_RDY		0x00000080
#define RAIN_DATA_RDY				0x00000100
#define WEATHER_CAL_DATA_RDY		0x00000200
#define POND_DATA_RDY				0x00000400
#define PH_CAL_DATA_RDY				0x00000800
#define BUTTON_DATA_RDY				0x00001000



#endif /* MAIN_STATION_DATA_TYPES_H_ */
