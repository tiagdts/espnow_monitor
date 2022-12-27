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

static char* TAG = "Solar Charger";

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

bool getSDdetect(void)
{
	if ( gpio_get_level(SD_DET) == 1 ) return false;
		else return true;
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
	gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(BLINK_GPIO, 1);

}

void hardwareReset(void)
{
	gpio_set_level(RESET_GPIO, 0);
	vTaskDelay(110 / portTICK_RATE_MS);
	gpio_set_level(RESET_GPIO, 1);
}

esp_err_t config_i2c( i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num )
{
	// configure the i2c controller 0 in master mode, normal speed
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
		if(i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
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
				fprintf(LogFile, "Time Stamp,Charge,Wiper,Peak Watts,Charge Volts,Charge Amps,Solar Volts,Charger Temperature\r\n");
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
	if( config_i2c( I2C_NUM_0, PIN_NUM_SDA, PIN_NUM_SCL )  != ESP_OK) printf("Configuration of bus 1 failed\r\n\r\n");
		else printf("Bus 1 configured\r\n\r\n");

}
