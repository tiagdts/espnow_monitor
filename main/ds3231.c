#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ds3231.h"

esp_err_t DS3231_read(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
	 if (size == 0) {
	        return ESP_OK;
	    }
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    // first, send device address (indicating write) & register to be read
	    i2c_master_write_byte(cmd, ( DS3231_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    // send register we want
	    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
	    // Send repeated start
	    i2c_master_start(cmd);
	    // now send device address (indicating read) & read data
	    i2c_master_write_byte(cmd, ( DS3231_ADDR << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
	    if (size > 1) {
	        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
	    }
	    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
	    i2c_master_stop(cmd);
	    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);
	    return ret;
}

esp_err_t DS3231_write(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    // first, send device address (indicating write) & register to be written
	    i2c_master_write_byte(cmd, ( DS3231_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    // send register we want
	    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
	    // write the data
	    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
	    i2c_master_stop(cmd);
	    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);
	    return ret;
}


uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}


esp_err_t ds3231_set_time(struct tm *time)
{

    uint8_t data[7];

    /* time/date data */
    data[0] = dec2bcd( time->tm_sec );
    data[1] = dec2bcd( time->tm_min );
    data[2] = dec2bcd( time->tm_hour );
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd( time->tm_wday + 1 );
    data[4] = dec2bcd( time->tm_mday );
    data[5] = dec2bcd( time->tm_mon + 1 );
    data[6] = dec2bcd( time->tm_year - 2000 );

    return DS3231_write(I2C_PORT,DS3231_ADDR_TIME, data, 7);
}

esp_err_t ds3231_get_raw_temp(int16_t *temp)
{
    uint8_t data[2];

    esp_err_t res = DS3231_read(I2C_PORT, DS3231_ADDR_TEMP, data, sizeof(data));
    if (res == ESP_OK)
        *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;

    return res;
}

esp_err_t ds3231_get_temp_integer(int8_t *temp)
{
    int16_t t_int;

    esp_err_t res = ds3231_get_raw_temp(&t_int);
    if (res == ESP_OK)
        *temp = t_int >> 2;

    return res;
}

esp_err_t ds3231_get_temp_float(float *temp)
{

    int16_t t_int;

    esp_err_t res = ds3231_get_raw_temp(&t_int);
    if (res == ESP_OK)
        *temp = t_int * 0.25;

    return res;
}

esp_err_t ds3231_get_time(struct tm *time)
{
    uint8_t data[7];

    /* read time */
    esp_err_t res = DS3231_read(I2C_PORT, DS3231_ADDR_TIME, data, 7);
        if (res != ESP_OK) return res;

    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG)
    {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    }
    else time->tm_hour = bcd2dec(data[2]); /* 24H */
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    //printf("Year : 0x%x\n", data[6] );
    time->tm_year = bcd2dec(data[6]) + 2000;
    //printf("Year : %u\n",bcd2dec(data[6]) );
    time->tm_isdst = 0;

    // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
    //applyTZ(time);

    return ESP_OK;
}

esp_err_t ds3231_get_pwr_status(bool *pwr_interruption)
{
    uint8_t data;

    /* read time */
    esp_err_t res = DS3231_read(I2C_PORT, DS3231_ADDR_STATUS, &data, 1);
        if (res != ESP_OK) return res;

    if( data & DS3231_STAT_OSF ) *pwr_interruption = true;
    else *pwr_interruption = false;

    return res;
}


esp_err_t ds3231_set_pwr_status(void)
{
    uint8_t data;

    /* read time */
    esp_err_t res = DS3231_read(I2C_PORT, DS3231_ADDR_STATUS, &data, 1);
        if (res != ESP_OK) return res;

    data = data & ~DS3231_STAT_OSF; // clear OSF bit, preserve other bits

    res = DS3231_write(I2C_PORT, DS3231_ADDR_STATUS, &data, 1);

    return res;
}
