#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include <esp_log.h>
#include "io.h"

// i2c Address
#define HD44780 0x27
#define SCROLL_DATA_COUNT	60
#define SCROLL_DATA_LEN		30
#define SCROLL_STR_LENGTH	512

#define	SCROLL_DATA_TYPE	0
#define	SCROLL_LOCATION		1
#define	SCROLL_MEASUREMENT	2
#define	SCROLL_LAST_UPDATED	3

#define LCD_DEGREE			0xdf
#define P_SYMBOL			0xf0
#define ARROW_SYMBOL		0x7e

typedef struct
{
	uint32_t data_type;
	uint32_t location;
	uint32_t measurement;
	time_t	store_time;
} scrollData_t;



void LCD_init(uint8_t addr, uint8_t cols, uint8_t rows);
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_home(void);
void LCD_clearScreen(void);
void LCD_writeChar(char c);
#define OLD_LCD_WRITESTR
#ifdef OLD_LCD_WRITESTR
void LCD_writeStr(char* str);
#else
void LCD_writeStr(char* str, uint8_t len );
#endif
void LCD_scroll_task(void *pvParameter);
void LCD_add_scroll_data(uint32_t type, uint32_t location,
		uint32_t measurement, time_t time, char *data);
void LCD_createSemaphores(void);
void LCD_buildScrollString( void );
void LCD_delete_scroll_data(uint32_t type, uint32_t location, uint32_t measurement);
