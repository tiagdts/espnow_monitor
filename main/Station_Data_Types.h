/*
 * Station_Data_Types.h
 *
 *  Created on: Jan 21, 2021
 *      Author: tiagd
 */

#ifndef MAIN_STATION_DATA_TYPES_H_
#define MAIN_STATION_DATA_TYPES_H_

#include <time.h>

#define CRC_FAIL					-1
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

	/* Locations inside */
#define LIVING_ROOM					1
#define KITCHEN						2
#define BEDROOM1					3
#define	BEDROOM2					4
#define SUNROOM						5
#define	UTILITY_ROOM				6
#define	HVAC_INSIDE_UNIT						7
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

typedef struct
{
	float temperature;
	float humidity;
	float baro_pressure;
	float wind_direction;
	float wind_velocity;
	uint16_t location_id;
} weatherData_t;

typedef struct
{
	float pump_temperature;
	float output_volume;
	float output_pressure;
	float output_rate;
	uint16_t location_id;
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

#define NO_DATA_RDY			0x00000000
#define WEATHER_DATA_RDY	0x00000001
#define PUMP_DATA_RDY		0x00000002
#define HVAC_DATA_RDY		0x00000004
#define PHONE_DATA_RDY		0X00000008
#define ROOM_DATA_RDY		0x00000010
#define TIME_DATA_RDY		0x00000020
#define MPPT_DATA_RDY		0x00000040



#endif /* MAIN_STATION_DATA_TYPES_H_ */
