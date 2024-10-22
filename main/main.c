
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
#include "esp_mac.h"

#include "io.h"
#include "espnow_.h"
//#include "ds3231.h"
#include "sntp_.h"
#include "protocol_examples_common.h"
#include "HD44780.h"
#include "loop_task.h"

//#define ESP_INTR_FLAG_DEFAULT 0



uint8_t address_count = 0;

void print_mac(const unsigned char *mac) {
	printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

/////

void print_current_time( time_t timestamp)
{
	printf( "current time %lld\n",timestamp );
}



/////
void get_mac(void)
{
    unsigned char mac_base[6] = {0};
    esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
    printf("MAC Address: ");
    print_mac(mac_base);
}


void app_main()
{


static	int32_t LED_count = 0;

//static int32_t update_calData = 0;
static	bool led_on = false;

//static weatherCalibrationData_t testCal;


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


 	LCD_init(HD44780, 16, 2);
	vTaskDelay(500 / portTICK_PERIOD_MS);
 	LCD_home();
 	LCD_clearScreen();
	LCD_setCursor(0, 0);
	LCD_writeStr(" Reset");


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


	// start wifi
	wifi_init();

	get_mac( );

	// start sensor network
	espnow_init();


	TaskHandle_t handle_loop_task = NULL;
	xTaskCreate(&loop_task, "loop_task", 4096, NULL, 2, &handle_loop_task );

    while(1)
    {


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

		LED_count++;

		vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
