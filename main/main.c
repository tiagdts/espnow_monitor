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


//#define INT 25
#define ESP_INTR_FLAG_DEFAULT 0

uint8_t address_count = 0;

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

void app_main()
{


	int32_t count = 0;
	bool led_on = false;

	init_GPIO( );

	// Initialize I2C port
	initI2C();


	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

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


	printf("scanning for I2C addresses\n");
	// see which addresses are active on the I2C bus
	address_count = scan_i2c( I2C_NUM_0, 0 );

	printf("Address Count = %u\n",address_count);

	// start wifi
	wifi_init();

	get_mac( );

	// start sensor network
	espnow_init();


    while(1)
    {
    	// Heartbeat LED
		count++;

		if( (count >= 3) && led_on )
		{
			/* Blink off (output low) */
			gpio_set_level(BLINK_GPIO, 0);
			led_on = false;
			count = 0;
		}
		else if( (count >= 3) && !led_on )
		{
			/* Blink on (output high) */
			gpio_set_level(BLINK_GPIO, 1);
			led_on = true;
			count = 0;
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
