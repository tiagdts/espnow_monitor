
/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "io.h"
#include "espnow_.h"
#include "ds3231.h"
#include "sntp_.h"


#define TIME_KEEPING
//#define INT 25
#define ESP_INTR_FLAG_DEFAULT 0

#define VANE_OFFSET				0.0
#define RAIN_GAUGE_CALIBRATION	0.029415 // inches per tip of rain gauge
#define ANEMOMETER_CALIBRATION	0.8888889 // counts per second to MPH
#define TEMPERATURE_CALIBRATION	0.0
#define HUMIDITY_CALIBRATION	0.0

typedef enum {
	// system setup types
	WEATHER_VANE_OFFSET	= 	0,
	RAIN_GAUGE_CAL 		= 	1,
	ANEMOMETER_CAL 		= 	2,
	TEMP_CAL			=	3,
	HUMIDITY_CAL		=	4,
	UNDEFINED			=	5
} sys_setup_types_t;


uint8_t address_count = 0;
#ifdef TIME_KEEPING
static time_t now = 0;
static struct tm timeinfo;
#endif
// setup/calibration values
static float setupValues[5] = { VANE_OFFSET, RAIN_GAUGE_CALIBRATION, ANEMOMETER_CALIBRATION,
											TEMPERATURE_CALIBRATION, HUMIDITY_CALIBRATION
							};

void setWeatherCalData(weatherCalibrationData_t *data )
{
	int32_t i;
	for(i=0;i<5;i++)
	{
		data->calibration_data[i] = setupValues[i];
	}
}

void print_mac(const unsigned char *mac) {
	printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

/////
void get_mac(void)
{
    unsigned char mac_base[6] = {0};
    esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
    printf("MAC Address: ");
    print_mac(mac_base);
}

void print_current_time( time_t timestamp)
{
	printf( "current time %ld\n",timestamp );
}

void update_display_time(void)
{


		timeData_t timeData;
		//#define MANUAL_UPDATE
		#ifdef MANUAL_UPDATE
		// second, minute, hour, weekday, day, month, year
		timeData.time_date[0] = 0; // seccond
		timeData.time_date[1] = 52; // minute
		timeData.time_date[2] = 14; // hour
		timeData.time_date[3] = 1; // weekday (1 = Sunday)
		timeData.time_date[4] = 12; // day
		timeData.time_date[5] = 3; // month
		timeData.time_date[6] = 23; // year
#else


		timeinfo = localtime( &now );

		/* time/date data */
		timeData.time_date[0] = dec2bcd( timeinfo.tm_sec );
		timeData.time_date[1] = dec2bcd( timeinfo.tm_min );
		timeData.time_date[2] = dec2bcd( timeinfo.tm_hour );
		/* The week data must be in the range 1 to 7, and to keep the start on the
		 * same day as for tm_wday have it start at 1 on Sunday. */
		timeData.time_date[3] = dec2bcd( timeinfo.tm_wday + 1 );
		timeData.time_date[4] = dec2bcd( timeinfo.tm_mday );
		timeData.time_date[5] = dec2bcd( timeinfo.tm_mon + 1 );
		timeData.time_date[6] = dec2bcd( timeinfo.tm_year - 100 );
#endif
		printf("set RTC to:%u,%u,%u,%u,%u,%u,%u\r",timeData.time_date[0],
				timeData.time_date[1],timeData.time_date[2],timeData.time_date[3],
				timeData.time_date[4],timeData.time_date[5],timeData.time_date[6]);

		// indicate new data
		timeData.setTime =  true;

		if( updateTimeloc( &timeData ) == 0 ) printf("Time Update Successful.\n");  // update succeeded
}

void app_main()
{


static	int32_t LED_count = 0;

static int32_t update_calData = 0;
static	bool led_on = false;

static weatherCalibrationData_t testCal;

#ifdef TIME_KEEPING
static	bool power_interruption  = false;
static int32_t update_time_count = 0;
static systemTimeData_t system_time;
#endif

// set time zone
	setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
	tzset();

	init_GPIO( );

		// Initialize I2C port
	initI2C();
	printf("scanning for I2C addresses\n");
	// see which addresses are active on the I2C bus
	address_count = scan_i2c( I2C_NUM_0, 0 );

	printf("Address Count = %u\n",address_count);


	#define NVS_INIT
	#ifdef NVS_INIT
		//Initialize NVS
		ESP_ERROR_CHECK( nvs_flash_init() );;
	#endif

#ifdef TIME_KEEPING
	ds3231_get_pwr_status( &power_interruption );
#define SNTP
#ifdef SNTP

#define SNTP_TASK
#ifdef SNTP_TASK
	TaskHandle_t handle_sntp_task = NULL;
	xTaskCreate(&sntp_task, "sntp_task", 4096, NULL, 3, &handle_sntp_task);

	while( get_sntp_busy() )
	{
		printf("waiting for NTPS time set\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	if( get_sntp_updated() )
	{
//		char strftime_buf[64];
//		time_t now = 0;
//		struct tm  *timeinfo;
		time(&now);

		print_current_time( now );
 //		printf("time now: %ld\n", now);
		// Set timezone to Eastern Standard Time and print local time
//		setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
//		tzset();
 //  		timeinfo = localtime( &now );
 //		strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//		printf("The current date/time is: %s", strftime_buf);
	}
#else
	if( power_interruption )
	{
		printf("get Web time\n");
		get_sntp( );
		set_RTC( );
		ds3231_set_pwr_status( );
	}
	else
	{
		printf("set system time from RTC\n");
		setsystemtime( );
	}
#endif

#endif

#endif





//#define SD_CARD
#ifdef SD_CARD
	// start SD Card
	if ( SD_CardStartUp() == ESP_OK )
	{
		printf("Opening Log File\n");
		openLogFile();
	}
	else printf("Log File Not Open\n");
#endif


	// start wifi
	wifi_init();

	get_mac( );

	// start sensor network
	espnow_init();

#ifdef TIME_KEEPING2
	time(&now);
	printf("time now: %ld\n", now);

	//check_RTC( now );
	//char strftime_buf[64];

	ds3231_get_time( &timeinfo );


	now = mktime( &timeinfo );
	printf("time now from RTC: %ld\n", now);


	printf("Month, Day, Year: %d-%d-%d\n",timeinfo.tm_mon,timeinfo.tm_mday,timeinfo.tm_year);
	//strftime(strftime_buf, sizeof(strftime_buf), "%c", timeinfo);
	//printf("The current date/time is: %s", strftime_buf);

#endif

    while(1)
    {
    	// Heartbeat LED
		LED_count++;
#ifdef TIME_KEEPING
		update_time_count++;
#endif
		update_calData++;

		if( (LED_count >= 3) && led_on )
		{
			/* Blink off (output low) */
			gpio_set_level(BLINK_GPIO, 0);
			led_on = false;
			LED_count = 0;
		}
		else if( (LED_count >= 3) && !led_on )
		{
			/* Blink on (output high) */
			gpio_set_level(BLINK_GPIO, 1);
			led_on = true;
			LED_count = 0;
		}
#ifdef TIME_KEEPING
		if( update_time_count >= 100 )
		{
			update_time_count = 0;
			time(&now);
			system_time.t.tv_sec = now;
			system_time.t.tv_usec = 0;
			sprintf( (char *)&system_time.description[0],"Epoch Unix Timestamp");
			if( updateSystemTimeloc(&system_time) == DATA_READ )
				printf("System Time sent to espnow: %s: %ld\n", system_time.description, system_time.t.tv_sec );
			else printf("System Time not updated\n");

			update_display_time( );

		}
#endif
 //#define UPDATE_CAL_DATA
#ifdef UPDATE_CAL_DATA
		if( update_calData >= 100 )
		{
			update_calData = 0;

			// change cal data
			setupValues[RAIN_GAUGE_CAL] = 0.043478;
			setupValues[WEATHER_VANE_OFFSET] = 56;
			printf("Weather vane offset %3.2f\n", setupValues[WEATHER_VANE_OFFSET] );
			setWeatherCalData( &testCal );
			if (setupValues[WEATHER_VANE_OFFSET] >= 360.0) setupValues[WEATHER_VANE_OFFSET] = 0.0;


			// send cal data to espnow
			if( updateWeatherCalLoc(&testCal) == DATA_READ )
				printf("Weather Calibration sent to espnow: Weather Vane Offset = %3.1f\n", setupValues[WEATHER_VANE_OFFSET] );
			else printf("Weather Calibration not updated\n");

		}
#endif
		vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
