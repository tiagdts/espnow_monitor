/*
 * RX8804.h
 *
 *  Created on: Oct 29, 2019
 *      Author: davidschmidt
 */

#ifndef RX8804_H_
#define RX8804_H_

//////////

#include <stdlib.h>

// I2C driver
#include "driver/i2c.h"

// used for timer
#include "freertos/task.h"

// Error library
#include "esp_err.h"

//////////




#include "driver/i2c.h"
#include "time.h"

#define ACK_CHECK_EN 0x01				/*!< I2C master will check ack from slave*/
#define NACK_VAL 0x01					/*!< I2C nack value */


// I2C Channel
#define RTC_CHANNEL 14

// Address
#define RX8804 0x32

// Registers

#define SEC	 			0x00		// Second
#define MINUTE			0x01		// Minute
#define HOUR		 	0x02		// Hour
#define WEEK 			0x03		// Day of Week
#define DAY				0x04		// Day	
#define MONTH			0X05		// Month
#define YEAR			0x06		// Year
#define RAM				0x07		// RAM
#define MIN_ALARM		0x08		// Minute Alarm
#define HOUR_ALARM		0X09		// Hour Alarm
#define WEEK_DAY_ALARM	0X0a		// Week/Day Alarm
#define COUNTER0		0x0b		// Preset value of Timer 0
#define COUNTER1		0x0c		// Preset value of Timer 1
#define EXTENSION_REG	0x0d		// Expansion Register
#define FLAG_REG		0x0e		// Flag Register
#define CONTROL_REG		0x0f		// Control Register	

#define TIME_STAMP_SEC	0x10		// Time Stamp Second
#define TIME_STAMP_MIN	0x11		// Time Stamp Minute
#define	TIME_STAMP_HR	0x12		// Time Stamp Hour
#define	TIME_STAMP_WK	0x13		// Time Stamp Week
#define TIME_STAMP_DAY	0x14		// Time Stamp Day
#define TIME_STAMP_MON	0x15		// Time Stamp Month
#define TIME_STAMP_YR	0x16		// Time Stamp Year
#define EVENT_CONTROL	0x17		// Event Control Register
#define EVENT_MONITOR	0x18		// Event Monitor Register
#define SOUT_CONTROL1	0x19		// SOUT CONTROL REGISTER 1
#define SOUT_CONTROL2	0x1a		// SOUT CONTROL REGISTER 2
#define TIMER_CONTROL	0x1b		// Timer Control Register
#define MONITOR_TIMER0	0x1c		// Current Timer 0 value
#define MONITOR_TIMER1	0x1d		// Current Timer 1 value
#define MONITOR_TIMER2	0x1e		// Current Timer 2 value
#define TIMER_COUNTER2	0x1f		// Preset value of Timer 2

// register bits

// Flag Register (FLAG_REG)

#define VDET	0x01
#define VLF		0x02
#define AF		0x08
#define TF		0x10
#define UF		0x20



#define VLF_READ_ERROR	0Xff  // there was an I2C error when trying to read VLF status

// Control Register bits (CONTROL_REG)
#define RESET	0x01
#define AIE		0x08			// alarm interrupt enable
#define TIE		0x10			// timer interrupt enable
#define UIE		0x20			// update interrupt enable
#define CSEL0	0x40			// compensation interval bit 0
#define CSEL1	0x80			// compensation interval bit 1

// extension Register bits
#define TSEL0	0x01			// Timer Source clock select 0
#define TSEL1	0x02			// Timer Source clock select 1
#define FSEL0	0x04			// Output frequency bit0
#define FSEL1	0x08			// Output frequency bit1
#define TE		0x10			// Timer enable
#define USEL	0x20			// Update rate 0 = 1 sec, 1 = 1 minute update rate
#define WADA	0x40			// Week alarm/Day alarm bit
#define TEST	0x80			// should always be 0

// Day of Week
#define SUNDAY		0x01
#define MONDAY		0x02
#define TUESDAY		0x04
#define WEDNESDAY	0x08
#define THURSDAY	0x10
#define FRIDAY		0x20
#define SATURDAY	0x40

esp_err_t RX8804_read(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_rd, size_t size);

esp_err_t RX8804_write(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size);

esp_err_t RX8804_init_update_intr(void);

esp_err_t RX8804_get_intr_status(uint8_t *status);

esp_err_t RX8804_clr_update_flag( void );

esp_err_t RX8804_stop_update_intr(void);

esp_err_t RX8804_start_update_intr(void);

esp_err_t RX8804_check_pwr_interruption(uint8_t *status);

uint8_t RX8804_time_to_tm(struct tm *timeinfo, uint8_t *timedata);

uint8_t RX8804_time_from_tm(struct tm timeinfo, uint8_t *timedata);

esp_err_t RX8804_set_time_date(uint8_t *time_date);

esp_err_t RX8804_get_time_date(uint8_t *time_date);

uint8_t  DecimalToBCD (uint8_t Decimal);

uint8_t  BCDToDecimal(uint8_t  BCD);

void setRTCstatus(uint8_t state);

void read_clock( void );

uint8_t getRTCstatus( void );

#endif /* RX8804_H_ */
