#pragma once

// i2c Address
#define HD44780 0x27

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
