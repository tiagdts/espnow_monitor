/*
 * loop_task.c
 *
 *  Created on: Oct 21, 2024
 *      Author: tiagd
 */
#include loop_task.h



uint8_t update_display(char* message, uint8_t line, uint8_t col )
{
	uint8_t len;
	LCD_setCursor(col, line);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	len = strlen( message )+1;
	LCD_writeStr(message);
	//len += 1;
	printf("update_display: %s, %u\n", message, len);
	return len;

}

void clear_line( uint8_t y_pos )
{
	LCD_setCursor(0, y_pos);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	//uint8_t len = strlen( message[ MSG_BLANK ] );
	LCD_writeStr(message[ MSG_BLANK]);

}

void loop_task(void *pvParameter)
{
	while(1)
	{
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}

}
