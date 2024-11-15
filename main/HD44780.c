
#include "HD44780.h"

// LCD module defines
#define LCD_LINEONE             0x00        // start of line 1
#define LCD_LINETWO             0x40        // start of line 2
#define LCD_LINETHREE           0x14        // start of line 3
#define LCD_LINEFOUR            0x54        // start of line 4

#define LCD_BACKLIGHT           0x08
#define LCD_ENABLE              0x04               
#define LCD_COMMAND             0x00
#define LCD_WRITE               0x01

#define LCD_SET_DDRAM_ADDR      0x80
#define LCD_READ_BF             0x40

// LCD instructions
#define LCD_CLEAR               0x01        // replace all characters with ASCII 'space'
#define LCD_HOME                0x02        // return cursor to first position on first line
#define LCD_ENTRY_MODE          0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF         0x08        // turn display off
#define LCD_DISPLAY_ON          0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET      0x30        // reset the LCD
#define LCD_FUNCTION_SET_4BIT   0x28        // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR          0x80        // set cursor position

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

char LCD_degStr[] = {LCD_DEGREE, 0};
char LCD_pctStr[] = {'%', 0 };
char LCD_pHstr[] = { P_SYMBOL, 'H', 0 };

static char tag[] = "LCD Driver";
static uint8_t LCD_addr;
static uint8_t LCD_cols;
static uint8_t LCD_rows;

static void LCD_writeNibble(uint8_t nibble, uint8_t mode);
static void LCD_writeByte(uint8_t data, uint8_t mode);
static void LCD_pulseEnable(uint8_t nibble);

static char DataToScroll[SCROLL_DATA_COUNT][SCROLL_DATA_LEN];

static scrollData_t ScrollDataInfo[SCROLL_DATA_COUNT];
		// 0: Data Type: e.g. "POND_DATA"
		// 1: Location: e.g. "POND"
		// 2: Measurement: e.g. "TEMPERATURE_DATA"
		// 3: Last Updated (seconds) - use to get rid of old data

static char scrollString[SCROLL_STR_LENGTH];
static uint16_t scrollFillPosition = 0;
static uint16_t scrollPosition = 0;

extern SemaphoreHandle_t xSemaphore_I2C;

SemaphoreHandle_t xSemaphore_LCD;

void LCD_createSemaphores(void)
{
	// create mutex semaphores to be used for LCD access
	// 	between Tasks
	xSemaphore_LCD = xSemaphoreCreateMutex();

}

void LCD_init(uint8_t addr, uint8_t cols, uint8_t rows)
{
    LCD_addr = addr;
    LCD_cols = cols;
    LCD_rows = rows;
    vTaskDelay(100 / portTICK_PERIOD_MS);                                 // Initial 40 mSec delay

    // Reset the LCD controller
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // First part of reset sequence
    vTaskDelay(10 / portTICK_PERIOD_MS);                                  // 4.1 mS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // second part of reset sequence
    ets_delay_us(400);                                                  // 100 uS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // Third time's a charm
    LCD_writeNibble(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                // Activate 4-bit mode
    ets_delay_us(2000);                                                   // 40 uS delay (min)

    // --- Busy flag now available ---
    // Function Set instruction
    LCD_writeByte(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                  // Set mode, lines, and font
    ets_delay_us(2000);

    // Clear Display instruction
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);                              // clear display RAM
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // Clearing memory takes a bit longer
    
    // Entry Mode Set instruction
    LCD_writeByte(LCD_ENTRY_MODE, LCD_COMMAND);                         // Set desired shift characteristics
    ets_delay_us(200);

    LCD_writeByte(LCD_DISPLAY_ON, LCD_COMMAND);                         // Ensure LCD is set to on
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
    if (row > LCD_rows - 1) {
        ESP_LOGE(tag, "Cannot write to row %d. Please select a row in the range (0, %d)", row, LCD_rows-1);
        row = LCD_rows - 1;
    }
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE, LCD_LINEFOUR};
    LCD_writeByte(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]), LCD_COMMAND);
}

void LCD_writeChar(char c)
{
    LCD_writeByte(c, LCD_WRITE);                                        // Write data to DDRAM
}
#define OLD_LCD_WRITESTR
#ifdef OLD_LCD_WRITESTR
void LCD_writeStr(char* str)
{
    while ( *str ) {
        LCD_writeChar(*str++);
    }
}
#else
void LCD_writeStr(char* str, uint8_t len )
{
    int32_t i;
    for(i=0;i<len;i++)
        LCD_writeChar(*str++);
}
#endif

void LCD_home(void)
{
    LCD_writeByte(LCD_HOME, LCD_COMMAND);
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // This command takes a while to complete
}

void LCD_clearScreen(void)
{
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);
    vTaskDelay(100 / portTICK_PERIOD_MS);                                   // This command takes a while to complete
}

#ifdef OLD_LCD
static void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
	// see if I2C bus is available, if it is take control of it
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		// Clock data into LCD
		LCD_pulseEnable(data);

		// give up control of I2C bus
		xSemaphoreGive( xSemaphore_I2C );
	}
}

static void LCD_pulseEnable(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data | LCD_ENABLE, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);  
    ets_delay_us(110);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data & ~LCD_ENABLE), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(500);
}

#else
static void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
	// see if I2C bus is available, if it is take control of it
	if( xSemaphoreTake( xSemaphore_I2C, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
	{
		uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1);
		i2c_master_write_byte(cmd, data, 1);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		// Clock data into LCD
		LCD_pulseEnable(data);

		// give up control of I2C bus
		xSemaphoreGive( xSemaphore_I2C );
	}
}




static void LCD_pulseEnable(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, data | LCD_ENABLE, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ets_delay_us(110);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, (data & ~LCD_ENABLE), 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ets_delay_us(500);
}
#endif

static void LCD_writeByte(uint8_t data, uint8_t mode)
{
    LCD_writeNibble(data & 0xF0, mode);
    LCD_writeNibble((data << 4) & 0xF0, mode);
}


void LCD_buildScrollString( void )
{
	char tmpStr[80];
	uint16_t i;

	// clear scroll string
	memset(scrollString,0,sizeof(scrollString));

	// check for data
	for( i=0; i<SCROLL_DATA_COUNT; i++ )
	{
		if( ScrollDataInfo[i].data_type != NO_DATA )
		{
			// create header

			// add location info
			switch( ScrollDataInfo[i].location )
			{
				case LIVING_ROOM:
						strcpy(tmpStr,"Living RM:");
					break;

				case KITCHEN:
						strcpy(tmpStr,"Kitchen:");
					break;

				case BEDROOM1:
						strcpy(tmpStr,"Bed RM1:");
					break;

				case BEDROOM2:
						strcpy(tmpStr,"Bed RM2:");
					break;

				case SUNROOM:
						strcpy(tmpStr,"Sun RM:");
					break;

				case UTILITY_ROOM:
						strcpy(tmpStr,"Utility RM:");
					break;

				case HVAC_INSIDE_UNIT:
					strcpy(tmpStr,"Air Handler:");
					break;

				case SHOP_FRONT:
						strcpy(tmpStr,"Shop Front:");
					break;

				case SHOP_BACK:
						strcpy(tmpStr,"Shop Back:");
					break;

				case DUCT_HALL:
						strcpy(tmpStr,"AC Vent Hall:");
					break;

				case DUCT_SUNROOM:
						strcpy(tmpStr,"AC Vent Sunrm:");
					break;

				case FRONT_YARD:
						strcpy(tmpStr,"Front Yard:");
					break;

				case BACK_YARD:
						strcpy(tmpStr,"Back Yard:");
					break;

				case WEST_SIDE:
						strcpy(tmpStr,"West Side:");
					break;

				case EAST_SIDE:
						strcpy(tmpStr,"East Side:");
					break;

				case GARAGE:
						strcpy(tmpStr,"Garage:");
					break;

				case HVAC_OUTSIDE_UNIT:
						strcpy(tmpStr,"Outside Unit:");
					break;

				case ROOF:
						strcpy(tmpStr,"Roof:");
					break;

				case POND:
						strcpy(tmpStr,"Pond:");
					break;
			}

			// add data
			strncat(tmpStr, &DataToScroll[i], SCROLL_DATA_LEN );

			// add units
			switch( ScrollDataInfo[i].measurement )
			{
				case TEMPERATURE_DATA:
						strncat(tmpStr,LCD_degStr, SCROLL_DATA_LEN);
					break;

				case HUMIDITY_DATA:
						strncat(tmpStr,LCD_pctStr, SCROLL_DATA_LEN);
					break;

				case BAROMETRIC_PRESSURE_DATA:
						strncat(tmpStr,"In Hg", SCROLL_DATA_LEN);
					break;

				case VOLUME_DATA:
						strncat(tmpStr,"Gal", SCROLL_DATA_LEN);
					break;

				case VELOCITY_DATA:
						strncat(tmpStr,"MPH", SCROLL_DATA_LEN);
					break;

				case DISTANCE_DATA:
						strncat(tmpStr,"Ft", SCROLL_DATA_LEN);
					break;

				case PRESSURE_DATA:
						strncat(tmpStr,"PSI", SCROLL_DATA_LEN);
					break;

				case INTENSITY_DATA:
					break;

				case CALLER_ID_DATA:
					break;

				case LOCATION_DATA:
					break;

				case FLOW_RATE_DATA:
						strncat(tmpStr,"GPH", SCROLL_DATA_LEN);
					break;

				case TEXT_DATA:
					break;

				case TIME_DATA:
					break;

				case VOLT_DATA:
						strncat(tmpStr,"V", SCROLL_DATA_LEN);
					break;

				case AMP_DATA:
						strncat(tmpStr,"A", SCROLL_DATA_LEN);
					break;

				case MASS_DATA:
						strncat(tmpStr,"LBS", SCROLL_DATA_LEN);
					break;

				case AREA_DATA:
						strncat(tmpStr,"Ft^2", SCROLL_DATA_LEN);
					break;

				case ANGLE_DATA:
						strncat(tmpStr,LCD_degStr, SCROLL_DATA_LEN);
					break;

			}
			// add comma
			strncat(tmpStr, ", ", SCROLL_DATA_LEN );
			if( (strlen(tmpStr) + strlen(scrollString) ) < SCROLL_STR_LENGTH )
				strcat(scrollString, tmpStr);
		}
	}
}

void LCD_add_scroll_data(uint32_t type, uint32_t location,
		uint32_t measurement, time_t time, char *data)
{
	int16_t i;
	bool updated = false;

	// check for existing entry
	for( i=0; i<SCROLL_DATA_COUNT; i++ )
	{
		if( ( ScrollDataInfo[i].data_type == type ) &&
			( ScrollDataInfo[i].location == location ) &&
			( ScrollDataInfo[i].measurement == measurement ) )
		{
			// update existing entry
			strncpy( &ScrollDataInfo[i], data, SCROLL_DATA_LEN );
			ScrollDataInfo[i].store_time = time;
			updated = true;
			return;
		}
	}

	// check for open spot
	for( i=0; i<SCROLL_DATA_COUNT; i++ )
	{
		if( ScrollDataInfo[i].data_type == NO_DATA )
		{
			ScrollDataInfo[i].data_type = type;
			ScrollDataInfo[i].location = location;
			ScrollDataInfo[i].measurement = measurement;
			ScrollDataInfo[i].store_time = time;
			strncpy( &ScrollDataInfo[i], data, SCROLL_DATA_LEN );
			return;
		}
	}

	// find oldest spot to store data
	time_t tmp_time = 0;
	int16_t record = -1;
	for( i=0; i<SCROLL_DATA_COUNT; i++ )
	{
		if( ScrollDataInfo[i].store_time < tmp_time )
		{
			tmp_time = ScrollDataInfo[i].store_time;
			record = i;
		}
	}

	if( record != -1 ) 	strncpy( &DataToScroll[record], data, SCROLL_DATA_LEN );

}

void LCD_scroll_task(void *pvParameter)
{
	printf("Scroll Task Started\n");
	memset( DataToScroll, 0, sizeof(DataToScroll) );
	memset( ScrollDataInfo, 0, sizeof(ScrollDataInfo) );
	// memset( scrollString, 0, sizeof(scrollString) );
	//strcpy(scrollString, "Pond Data: 1729021288, 257, -100.00, 22.44, 15, 15813, 346331, 347590, -100.000, 0.000, 9.599");
	strcpy(scrollString, "Pond Data: 1729021288, 257, ");

	uint16_t i;
	char displayStr[LCD_cols];
	uint16_t len;

	while(1)
	{
		if( strlen(scrollString) != 0 )
		{
			if( strlen(scrollString) < ( LCD_cols-1 ) )
			{
				// no scroll needed
				strcpy(displayStr,scrollString);
			}
			else
			{
				strncpy(displayStr, &scrollString[scrollPosition], LCD_cols-1 );
				displayStr[LCD_cols-1] = 0;

				len = strlen(displayStr);

				if( len < (LCD_cols-2) )
				{
					strcat(displayStr,"|");
					len++;
					strncat( &displayStr[ len ], scrollString, ( (LCD_cols-1) - len ) );
					displayStr[LCD_cols-1] = 0;
				}

				if( strlen( scrollString) > (LCD_cols-1) )
				{
					scrollPosition++;
					if( scrollPosition > strlen(scrollString) ) scrollPosition = 0;
				}
			}
			if( xSemaphoreTake( xSemaphore_LCD, TASK_WAIT_TIME / portTICK_PERIOD_MS ) == pdTRUE )
			{
				LCD_setCursor(0, 1);
				vTaskDelay(10 / portTICK_PERIOD_MS);
				LCD_writeStr(displayStr);
				// give up control of LCD
				xSemaphoreGive( xSemaphore_LCD );
			}
		}
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}
