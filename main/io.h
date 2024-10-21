/*
 * io.h
 *
 *  Created on: Nov 28, 2019
 *      Author: David
 */



#ifndef MAIN_IO_H_
#define MAIN_IO_H_

#include <stdio.h>
#include <stdlib.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

// I2C driver
#include "driver/i2c.h"

// Error library
#include "esp_err.h"

// pwm
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"

// SD card and FAT
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "sys/stat.h"

#include "stdio.h"
#include "sys/unistd.h"
#include "sys/stat.h"
#include "dirent.h"
#include "string.h"

// REQUIRED DEVICE lIST HEADER FILES
#ifdef IO_CHIPS
#include "PCAL9554C.h"
#include "ADS1115.h"
#include "AD5245.h"
#include "LTC2942.h"
#include "PCT2075.h"
#include "HDC3020.h"
#endif

#define EOL				255	// End Of List - end of required device list for a complete system

#define MOUNT_POINT "/sdcard"
#define SPI_DMA_CHAN    1

#define RESET_GPIO		32	// peripheral reset
#define INT				39  // interrupt pin from PCAL9554


#define HEARTBEAT_LED		32
#define PWR_ENA_GPIO	26
#define PULSE			39

// I2C pins
#define PIN_NUM_SDA 	21
#define PIN_NUM_SCL		22
#define I2C_PORT		I2C_NUM_0

// SD Card pins
#define PIN_NUM_MISO	19
#define PIN_NUM_MOSI	23
#define PIN_NUM_CLK		18
#define PIN_NUM_CS		5
#define SD_DET			34

//#define TASK_DATA_WAIT_TIME 100
#define TASK_WAIT_TIME		300

#define BUTTON_PRESS			25
#define PH_CAL_MODE_CLR			32
#define PH_CAL_MODE				26		// Active High signal


bool getSDdetect(void);
uint8_t getValidAddress(uint8_t bus, uint8_t address);
bool getFatAvailable(void);
bool getLogAvailable(void);
FILE *getLogFileHandle(void);
void init_GPIO( void );
void ConfigurePWM(void);
void hardwareReset(void);
esp_err_t config_i2c( i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num );
uint8_t scan_i2c( i2c_port_t i2c_num, uint8_t bus );
esp_err_t SD_CardStartUp();
void openLogFile(void);
void closeLogFile(void);
void openLaneFile(void);
void openStatusFile(void);
void initI2C(void);
void fatfs_opendir(const char* path);

bool checkDevices(void);

#endif /* MAIN_IO_H_ */
