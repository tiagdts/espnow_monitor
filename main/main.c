
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
#include "protocol_examples_common.h"
#include "HD44780.h"
#include "pH_calibrate.h"


//#define TIME_KEEPING
//#define INT 25
#define ESP_INTR_FLAG_DEFAULT 0

#define VANE_OFFSET				0.0
#define RAIN_GAUGE_CALIBRATION	0.029415 // inches per tip of rain gauge
#define ANEMOMETER_CALIBRATION	0.8888889 // counts per second to MPH
#define TEMPERATURE_CALIBRATION	0.0
#define HUMIDITY_CALIBRATION	0.0

// Day of Week
#define SUNDAY		0x01
#define MONDAY		0x02
#define TUESDAY		0x04
#define WEDNESDAY	0x08
#define THURSDAY	0x10
#define FRIDAY		0x20
#define SATURDAY	0x40

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
static struct tm *timeinfo;
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
/*
/////
void get_mac(void)
{
    unsigned char mac_base[6] = {0};
    esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
    printf("MAC Address: ");
    print_mac(mac_base);
}
*/
void print_current_time( time_t timestamp)
{
	printf( "current time %lld\n",timestamp );
}

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

#ifdef TIME_KEEPING
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

		RX8804_time_from_tm(timeinfo, &timeData);



		/* time/date data */
//		timeData.time_date[0] = dec2bcd( timeinfo.tm_sec );
//		timeData.time_date[1] = dec2bcd( timeinfo.tm_min );
//		timeData.time_date[2] = dec2bcd( timeinfo.tm_hour );
		/* The week data must be in the range 1 to 7, and to keep the start on the
		 * same day as for tm_wday have it start at 1 on Sunday. */
//		timeData.time_date[3] = dec2bcd( timeinfo.tm_wday + 1 );
//		timeData.time_date[4] = dec2bcd( timeinfo.tm_mday );
//		timeData.time_date[5] = dec2bcd( timeinfo.tm_mon + 1 );
//		timeData.time_date[6] = dec2bcd( timeinfo.tm_year - 100 );
#endif
		printf("set RTC to (BDC):%x,%x,%x,%x,%x,%x,%x\n",timeData.time_date[0],
				timeData.time_date[1],timeData.time_date[2],timeData.time_date[3],
				timeData.time_date[4],timeData.time_date[5],timeData.time_date[6]);

		// indicate new data
		timeData.setTime =  true;

		if( updateTimeloc( &timeData ) == 0 ) printf("Time Update Successful.\n");  // update succeeded
}
#endif

void app_main()
{


static	int32_t LED_count = 0;

static int32_t update_calData = 0;
static	bool led_on = false;

//static weatherCalibrationData_t testCal;

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


 	LCD_init(HD44780, 16, 4);
 	LCD_home();
 	LCD_clearScreen();


	#define NVS_INIT
	#ifdef NVS_INIT
	 // Initialize NVS
	    esp_err_t ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        ESP_ERROR_CHECK( nvs_flash_erase() );
	        ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK( ret );

	#endif

#ifdef TIME_KEEPING
	ds3231_get_pwr_status( &power_interruption );
#define SNTP
#ifdef SNTP

#define SNTP_TASK
#ifdef SNTP_TASK

    obtain_time();

	time(&now);

	print_current_time( now );

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

	//get_mac( );

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

//#define SEND_START_BUTTON

#ifdef SEND_START_BUTTON

	buttonData_t button;
	button.time = 100;
	button.button_type = BUTTON_TYPE_PH_CAL;
	button.button_data = START_BTN;
	button.location_id = POND;

#endif

	TaskHandle_t handle_calibration_task = NULL;
	xTaskCreate(&calibration_Task, "pH_Calibrate_task", 2048, NULL, 2, &handle_calibration_task );

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
			gpio_set_level(HEARTBEAT_LED, 0);
			led_on = false;
			LED_count = 0;
		}
		else if( (LED_count >= 3) && !led_on )
		{
			/* Blink on (output high) */
			gpio_set_level(HEARTBEAT_LED, 1);
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
				printf("System Time sent to espnow: %s: %lld\n", system_time.description, system_time.t.tv_sec );
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



#ifdef SEND_START_BUTTON
		// send button start data to espnow
		if( updateButtonloc(&button) == DATA_READ )
			printf("Button start sent\n");
		else printf("Button start not sent\n");

#endif
		vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
