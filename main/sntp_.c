/*
 *  sntp_.c
 *
 *  Created on: Jan 1, 2023
 *      Author: tiagd
 */


#include "sntp_.h"

static void obtain_time(void);
static void initialize_sntp(void);

static bool time_sync = false;
static const char *TAG = "sntp";
static bool sntp_updated = false;
static bool sntp_busy = true;

bool get_sntp_updated( void )
{
	return sntp_updated;
}

bool get_sntp_busy( void )
{
	return sntp_busy;
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    time_sync = true;
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /**
     * NTP server address could be aquired via DHCP,
     * see LWIP_DHCP_GET_NTP_SRV menuconfig option
     */
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);
#endif

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    //time_t now = 0;
    //struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    //time(&now);
    //localtime_r(&now, &timeinfo);

    ESP_ERROR_CHECK( example_disconnect() );
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

bool get_sntp(void)
{
	//time_t now;
	time_sync = false;
    obtain_time();
    // update 'now' variable with current time
    //time(&now);
    sntp_updated = true;
    return time_sync;
}

void sntp_task(void *pvParameter)
{
	sntp_busy = true;
	sntp_updated = false;
	while(1)
	{
		if( sntp_busy )
		{
			sntp_updated = get_sntp( );
			sntp_busy = false;
		}
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}
