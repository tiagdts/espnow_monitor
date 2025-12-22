/*
 * io.c
 *
 *  Created on: Nov 28, 2019
 *      Author: David
 */

#include "io.h"



static bool FAT_Available = false;
static bool LogData = false;
static FILE *LogFile = NULL;
static uint8_t i2cBus[2][127];
static time_t last_mppt_time = 0;
static time_t last_rain_time = 0;
static time_t last_weather_time = 0;
static time_t last_pond_time = 0;

uint8_t rx_data[BUF_SIZE + 1];

static char* TAG = "Solar Charger";


static QueueHandle_t tx_queue;

SemaphoreHandle_t xSemaphore_I2C;
SemaphoreHandle_t xSemaphore_uart;

static const char *commandHeader[] = {
										"SYSTEM_TIME",
										"RESET"
									};

void io_createSemaphores(void)
{
	// create mutex semaphores to be used for I2C bus access and uart data access
	// 	between Tasks
	xSemaphore_I2C = xSemaphoreCreateMutex();
	xSemaphore_uart = xSemaphoreCreateMutex();

}


#ifdef IO_CHIPS
static uint8_t requiredDevice[] = {AD5245_1, PCAL9554, ADS1115, LTC2942, HDC3020, PCT2075, EOL};
// See if all the required devices are present
bool checkDevices(void)
{
	int32_t i = 0;

	while( requiredDevice[i] != 255 )
	{
		if( i2cBus[0][requiredDevice[i++]] != 1) return false;
	}
	return true;
}
#endif

bool checkExtention(char *extension)
{
	uint8_t count = 0;

	const char extensions[3][5] = {".jsm", ".htm", ".js" };

	for(count=0; count<3; count++)
	{
		if (strcasecmp(extension, extensions[count] ) == 0) return true;
	}

	return false;
}


bool saveFile(const char *name)
{
	char fileName[35];
	size_t inCount = 0, outCount = 0;
	bool done = false;
	bool success = true;
	int inStatus, outStatus;

	//memset( fileName,0,sizeof(fileName) );

	strcpy(fileName, "/spiffs/");
	strcat(fileName, name);
	FILE* fout = fopen(fileName, "wb");
	if (fout == NULL)
	{
		return false;
	}

	strcpy(fileName, "/sdcard/");
	strcat(fileName, name);
	FILE* fin = fopen(fileName, "rb");
	if (fin == NULL)
	{
		return false;
	}

	char buffer[80];

	while(!done)
	{
		// write data from SD card to internal file system
		inCount = fread(buffer, 1, sizeof(buffer), fin);

		if(inCount != 0)
			outCount = fwrite(buffer, 1, inCount, fout);

		if(inCount != outCount) // write failed
		{
			done = true; // quit
			success = false;
		}

		if( feof(fin) != 0) done = true;

	}

	inStatus = fclose(fin);
	outStatus = fclose(fout);

	if( (inStatus != 0) || (outStatus != 0) || (success == false) ) return false; // something failed
		else return true;

}

uint8_t getValidAddress(uint8_t bus, uint8_t address)
{
	return i2cBus[bus][address];
}

bool getFatAvailable(void)
{
	return FAT_Available;
}

bool getLogAvailable(void)
{
	return LogData;
}

FILE *getLogFileHandle(void)
{
	return LogFile;
}

#define SD_DETECT
#ifdef SD_DETECT
bool getSDdetect(void)
{
	if ( gpio_get_level(SD_DET) == 1 ) return false;
		else return true;
}
#endif

void force_cal_mode_off(void)
{
	gpio_set_level(PH_CAL_MODE_CLR, 0);
	vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(PH_CAL_MODE_CLR, 1);
}


void init_GPIO( void )
{
#ifdef OTHER_IO
	gpio_pad_select_gpio(RESET_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RESET_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(RESET_GPIO, 1);

	gpio_pad_select_gpio(RESET_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RESET_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(RESET_GPIO, 1);

	// SD CARD DETECT SIGNAL
	gpio_pad_select_gpio(SD_DET);
	/* Set the GPIO as an input */
	gpio_set_direction(SD_DET, GPIO_MODE_INPUT);


	// configure interrupt pins as GPIO pins
	gpio_pad_select_gpio(INT2);

	// set the correct direction
	gpio_set_direction(INT2, GPIO_MODE_INPUT);

	// enable interrupt on falling (1->0) edge for pin
	gpio_set_intr_type(INT2, GPIO_INTR_NEGEDGE);


	// MAX31865 /DRDY  SIGNAL
	gpio_pad_select_gpio(MAX31865_DRDY_GPIO);
	/* Set the GPIO as an input */
	gpio_set_direction(MAX31865_DRDY_GPIO, GPIO_MODE_INPUT);
	// enable interrupt on falling (1->0) edge for pin
	gpio_set_intr_type(MAX31865_DRDY_GPIO, GPIO_INTR_NEGEDGE);

	gpio_pad_select_gpio(PWR_ENA_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(PWR_ENA_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(PWR_ENA_GPIO, 1);

	// configure interrupt pin as GPIO pins
	gpio_pad_select_gpio(INT);
	// set the correct direction
	gpio_set_direction(INT, GPIO_MODE_INPUT);
	// enable pull-up
	gpio_set_pull_mode(INT, GPIO_PULLUP_ONLY);
	// enable interrupt on falling (1->0) edge for pin
	gpio_set_intr_type(INT, GPIO_INTR_NEGEDGE);


#endif
	// Heartbeat
	esp_rom_gpio_pad_select_gpio(HEARTBEAT_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(HEARTBEAT_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(HEARTBEAT_LED, 1);

}

void hardwareReset(void)
{
	gpio_set_level(RESET_GPIO, 0);
	vTaskDelay(110 / portTICK_PERIOD_MS);
	gpio_set_level(RESET_GPIO, 1);
}

esp_err_t config_i2c( i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num )
{
	// configure the i2c controller "i2c_num" in master mode, normal speed
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda_io_num; //18;  23
	conf.scl_io_num = scl_io_num; //19;  22
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(i2c_num, &conf);
	return i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
}

uint8_t scan_i2c( i2c_port_t i2c_num, uint8_t bus )
{
	//printf("scanning the bus...\r\n\r\n");
	int devices_found = 0;

	for(int address = 1; address < 127; address++)
	{
		i2cBus[bus][address] = 0;
		// create and execute the command link
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
			i2cBus[bus][address] = 1;
			devices_found++;
		}
		i2c_cmd_link_delete(cmd);
	}
	if(devices_found == 0) printf("\r\n-> no devices found\r\n");
	printf("\r\n...I2C scan completed!\r\n");
	return devices_found;
}


esp_err_t SD_CardStartUp()
{
    esp_err_t ret;

	  // Options for mounting the filesystem.
	  // If format_if_mount_failed is set to true, SD card will be partitioned and
	  // formatted in case when mounting fails.
	  esp_vfs_fat_sdmmc_mount_config_t mount_config = {

			  .format_if_mount_failed = false,
		   // EXAMPLE_FORMAT_IF_MOUNT_FAILED
			.max_files = 5,
		  .allocation_unit_size = 16 * 1024
	  };
	  sdmmc_card_t *card;
	  const char mount_point[] = MOUNT_POINT;
	  ESP_LOGI(TAG, "Initializing SD card");

	  // Use settings defined above to initialize SD card and mount FAT filesystem.
	  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
	  // Please check its source code and implement error recovery when developing
	  // production applications.
	  ESP_LOGI(TAG, "Using SPI peripheral");

	  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	  spi_bus_config_t bus_cfg = {
		  .mosi_io_num = PIN_NUM_MOSI,
		  .miso_io_num = PIN_NUM_MISO,
		  .sclk_io_num = PIN_NUM_CLK,
		  .quadwp_io_num = -1,
		  .quadhd_io_num = -1,
		  .max_transfer_sz = 4000,
	  };
	  ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
	  if (ret != ESP_OK) {
		  ESP_LOGE(TAG, "Failed to initialize bus.");
		  return ret;
	  }

	  // This initializes the slot without card detect (CD) and write protect (WP) signals.
	  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
	  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	  slot_config.gpio_cs = PIN_NUM_CS;
	  slot_config.host_id = host.slot;

	  ESP_LOGI(TAG, "Mounting filesystem");
	  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

	  if (ret != ESP_OK) {
		  if (ret == ESP_FAIL) {
			  ESP_LOGE(TAG, "Failed to mount filesystem. "
					   "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
		  } else {
			  ESP_LOGE(TAG, "Failed to initialize the card (%s). "
					   "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
		  }
		  return ret;
	  }
	  ESP_LOGI(TAG, "Filesystem mounted");

	  // Card has been initialized, print its properties
	  sdmmc_card_print_info(stdout, card);

	FAT_Available = true;  // storage available
	return ret;
}

void openLogFile(void)
{
	struct stat st;

	LogData = false;
	if(FAT_Available == true)
	{
		if (stat("/sdcard/logdata.txt", &st) == 0)
		{
			// Open it if it exists
			ESP_LOGI(TAG, "Open log file for appending data");
			LogFile = fopen("/sdcard/logdata.txt", "a");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open log file for appending");
			}
			else
			{
				fclose(LogFile);
				LogData = true;
			}
		}
		else
		{
			ESP_LOGI(TAG, "Creating log file");
			LogFile = fopen("/sdcard/logdata.txt", "w");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open log file for writing");
			}
			else
			{
				//fprintf(LogFile, "Time Stamp,Charge,Wiper,Peak Watts,Charge Volts,Charge Amps,Solar Volts,Charger Temperature\r\n");
				fclose(LogFile);
				LogData = true;
			}
		}
	}
}

void openLaneFile(void)
{
	struct stat st;

	if(FAT_Available == true)
	{
		if (stat("/sdcard/lanedata.txt", &st) == 0)
		{
			// Open it if it exists
			ESP_LOGI(TAG, "Open Lane data file for appending data");
			LogFile = fopen("/sdcard/lanedata.txt", "a");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open lane data file for appending");
			}
			else
			{
				fclose(LogFile);
			}
		}
		else
		{
			ESP_LOGI(TAG, "Creating Lane data file");
			LogFile = fopen("/sdcard/lanedata.txt", "w");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open lane data file for writing");
			}
			else
			{
				fprintf(LogFile,"Time Stamp,Lane 1,Lane 2,Lane 3,Lane 4,Lane 5,Lane 6,Lane 7,Lane 8,Lane 9,Lane 10,Lane 11,Lane 12,Lane 13,Lane 14\r\n");
				fclose(LogFile);
			}
		}
	}
}

void openStatusFile(void)
{
	struct stat st;

	if(FAT_Available == true)
	{
		if (stat("/sdcard/statdata.txt", &st) == 0)
		{
			// Open it if it exists
			ESP_LOGI(TAG, "Open Status data file for appending data");
			LogFile = fopen("/sdcard/statdata.txt", "a");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open ststus data file for appending");
			}
			else
			{
				fclose(LogFile);
			}
		}
		else
		{
			ESP_LOGI(TAG, "Creating Status data file");
			LogFile = fopen("/sdcard/statdata.txt", "w");
			if (LogFile == NULL) {
				ESP_LOGE(TAG, "Failed to open Status data file for writing");
			}
			else
			{
				fprintf(LogFile,"Time Stamp,Bees In,Bees Out,Temp,Batt Amps,Batt Volts,Solar Volts\r\n");
				fclose(LogFile);
			}
		}
	}
}

void closeLogFile(void)
{
	if(FAT_Available == true)
	{
		fclose(LogFile);
	}
}

void fatfs_opendir(const char* path)
{
    const char * file_name = "logdata.txt";
    DIR* dir = opendir(path);
    char *ext;

    if( dir != NULL )
    {
		while (true) {
			struct dirent* de = readdir(dir);
			if (!de) {
				break;
			}
			if (strcasecmp(de->d_name, file_name) != 0)  // don't copy the log file
			{
				// see if it has an extension and check to see if we want to save it
				ext = strrchr(de->d_name, '.');
				if(ext != NULL) // if it has an extenstion check to see if it is on the list
					if( checkExtention(ext) ) saveFile(de->d_name);
			}
		}
		closedir(dir);
    }
}

// initialize i2c ports
void initI2C(void)
{
	// initialize the I2C ports
	if( config_i2c( I2C_NUM_0, PIN_NUM_SDA, PIN_NUM_SCL )  != ESP_OK )
		printf("Configuration of bus 1 failed\r\n\r\n");
	else printf("Bus 1 configured\r\n\r\n");

	// create I2 mutex semaphores
	io_createSemaphores();

}



void tx_task(void *arg) {
    char *tx_data;
    while (1) {
        // Wait indefinitely for data to arrive in the queue
        if (xQueueReceive(tx_queue, &tx_data, portMAX_DELAY) == pdPASS) {
            // Once data is received, transmit it via UART
            uart_write_bytes(UART_NUM, tx_data, strlen(tx_data));
            // Free the allocated memory after transmission
            free(tx_data);
        }
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    //uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1)
    {
		if( xSemaphoreTake( xSemaphore_uart, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
		{
	        const int rxBytes = uart_read_bytes(UART_NUM_1, rx_data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
	        if (rxBytes > 0) {
	            rx_data[rxBytes] = 0;
	            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, rx_data);
	            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, rx_data, rxBytes, ESP_LOG_INFO);
	            // System time
	            if (rxBytes < 80)
	            	check_uart_command((char*)(rx_data) );
	            
	        }
	        else rx_data[0] = 0;
	        xSemaphoreGive( xSemaphore_uart );
	    }
	    // check for data every second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // free(data);
}

bool addStrToUartQueue(char *str_to_send)
{
	 // Send the pointer to the tx_task
	 //printf("1-%s", str_to_send);
	 bool result = false;
	 char *out_str = strdup(str_to_send);
	 
	 if( out_str != NULL )
	 {
		 //printf("2-%s", out_str);
		 if( tx_queue != NULL )
		 {
		    if (xQueueSend(tx_queue, &out_str, portMAX_DELAY) != pdPASS)
		    {
		        // Handle error if queue is full
		        ESP_LOGE("IO", "Failed to send to TX queue");
		        result = false;
		    } else result = true;
		 } else	result = false;
		 
		//free(out_str);
	 }
	 return result;
}

// initailize uart
void initUart(void)
{
    // 1. Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // 2. Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TXD, UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 3. Install UART driver, but without an internal TX buffer (handled by our queue/task)
    // The TX buffer size is set to 0, so uart_write_bytes blocks until data is sent to the FIFO
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    // 4. Create the custom queue that holds pointers to strings
   	tx_queue = xQueueCreate(20, sizeof(char *));
   	
   	// Check if the creation was successful
	if( tx_queue != NULL )
	{
		printf("uart Queue created\n");
		/*
		char *test_str = strdup("Hello world");
    	// Queue was created successfully
    	if (xQueueSend(tx_queue, &test_str, portMAX_DELAY) != pdPASS)
	    {
	        // Handle error if queue is full
	        ESP_LOGE("IO", "Failed to send to TX queue");
	    }
		*/
	}
	else
	{
 	   // Queue was not created successfully
 		printf("uart Queue created failed\n");
	}
   	
   	
    
    xSemaphore_uart = xSemaphoreCreateMutex();
    
     // 5. Create the transmission and receive tasks
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, 10, NULL);
    xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, 10, NULL);
}

bool checkForUartData(char *data)
{
	if( xSemaphoreTake( xSemaphore_uart, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		if( strlen( (const char*)(rx_data) ) != 0)
		{
			strcpy( data,(const char*)(rx_data) );
			return true;
		}
		else return false;
		
		xSemaphoreGive( xSemaphore_uart );
	}
	else return false;
}

void strToUpper(uint8_t *data)
{
	uint32_t i;

	for(i=0; i < strlen(  (char *) data); i++)
		data[i] = toupper( (int) data[i] );
}

const char *getCommand( uart_command_types_t id )
{
	return commandHeader[id];
}

uart_command_types_t checkCmd(char *inStr)
{
	//sys_setup_types_t headerType = UNDEFINED;
	strToUpper( (uint8_t *)(inStr));
	uart_command_types_t i;
	for(i=SYSTEM_TIME;i<UNDEFINED;i++)
	{
		if( strcmp( inStr, getCommand(i) ) == 0 ) return i;
	}
	return i;

}


bool check_uart_command(char *cmd)
{
	double value = 0;
	uart_command_types_t cmdType;
	char cmdStr[20] = {0};
	
	
	int count =  sscanf(cmd, "%s %lf", cmdStr, &value);
	//printf("header: %s, value: %f\n", headerStr, value);
	if( count == 2 )
	{
		cmdType = checkCmd(cmdStr);
		if( cmdType != UNDEFINED )
		{
			 printf("%s %10.0lf\n",cmdStr, value);
			 
			 switch(cmdType)
			 {
				 case SYSTEM_TIME:
				 		systemTimeData_t data;
				 		data.t.tv_sec = value;
						data.t.tv_usec = 0;
						sprintf( (char *)&data.description[0],"Epoch Unix Timestamp-Pi");
						if( updateSystemTimeloc(&data) == DATA_READ )
							printf("System Time sent to espnow: %s: %lld\n", data.description, data.t.tv_sec );
						else printf("System Time not updated\n");	
				 	break;
				 	
				 case RESET:
				 	break;
				 	
				 default:
				 
			 }
			 
		}
		return true;
	}
	return false;
}

esp_err_t log_data( void *data, uint8_t dataType )
{
	esp_err_t ret = ESP_OK;
	weatherData_t weatherData;
	rainData_t	rainData;
	pondData_t pondData;
	MPPTdata_t MPPTdata;
	bool saveToFile = false;
	char tmpstr[150];
	FILE *fileOut;

	switch( dataType )
	{
		case WEATHER_DATA :
				weatherData = *( weatherData_t *) data;
				if(last_weather_time != weatherData.time )
				{
					last_weather_time = weatherData.time;
					sprintf(tmpstr,"Weather Data, %lld, %d, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f\n",
							weatherData.time,  weatherData.location_id, weatherData.baro_pressure,
							weatherData.humidity,  weatherData.temperature, weatherData.wind_direction,
							weatherData.wind_velocity);
					saveToFile = true;
				}
				else saveToFile = false;

			break;

		case POND_DATA :
				pondData = *( pondData_t *) data;
				if(last_pond_time != pondData.time )
				{
					last_pond_time = pondData.time;
					last_pond_time = pondData.time;
					sprintf(tmpstr,"Pond Data, %lld, %d, %2.2f, %2.2f, %d, %u, %lu, %lu, %3.3f, %3.3f, %3.3f\n",
							 pondData.time,  pondData.location_id,  pondData.air_temperature,
								 pondData.water_temperature,  pondData.hour,  pondData.light_level,
								 pondData.hourly_light_accum,  pondData.daily_light_accum,
								 pondData.turbidity,  pondData.fluoresence,  pondData.pH);
					saveToFile = true;
				}
				else saveToFile = false;

			break;

		case MPPT_DATA :
				MPPTdata = *( MPPTdata_t *) data;
				if(last_mppt_time != MPPTdata.time )
				{
					last_mppt_time = MPPTdata.time;
					sprintf(tmpstr, "MPPT Data, %lld, %u, %d, %u, ,%u, %3.2f, %3.2f, %3.2f, %3.2f, %3.1f\n",
							 MPPTdata.time,  MPPTdata.new_data,  MPPTdata.wiper,  MPPTdata.location_id,
							 MPPTdata.charge,  MPPTdata.peak_charge_current,  MPPTdata.peak_charge_volts,
							 MPPTdata.peak_watts,  MPPTdata.peak_solar_volts,  MPPTdata.charger_temp);
					saveToFile = true;
				}
				else saveToFile = false;
			break;

		case RAIN_DATA :
				rainData = *( rainData_t *) data;
				if(last_rain_time != rainData.time )
				{
					last_rain_time = rainData.time;
					sprintf(tmpstr, "Rain Data, %lld, %d, %u, %2.2f, %2.2f, %3.2f\n",  rainData.time,
							 rainData.location_id,  rainData.hour,  rainData.accumulation_1hour,
							 rainData.accumulation_24hour,  rainData.rate);
					saveToFile = true;
				}
				else saveToFile = false;
			break;

		default:
			saveToFile = false;
	}

	// see if data should be saved
	if( saveToFile )
	{
		// see if SD card is in available
		if( getSDdetect() )
		{
			// open file if storage is available
			if( getFatAvailable( ) )
			{
				fileOut = fopen("/sdcard/logdata.txt", "a");
				if (fileOut != NULL)
				{
					// save data to SD card
					fprintf(fileOut, "%s", tmpstr);
					fclose(fileOut);
				}
				else ret = ESP_FAIL;
			}
			else ret = ESP_FAIL;
		 }
		 else ret = ESP_FAIL;
	}

	return ret;
}

