#include <avr/io.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "uart.h"
#include "hd44780.h"

#define LCD_ROWS 4
#define LCD_COLS 20

#define PIN_RS 10
#define PIN_RW 11
#define PIN_EN 12

#define PIN_DATA0 15
#define PIN_DATA1 16
#define PIN_DATA2 17
#define PIN_DATA3 18
#define PIN_DATA4 4
#define PIN_DATA5 5
#define PIN_DATA6 6
#define PIN_DATA7 7

int main(void)
	{
        uart_init_115200();           // initialize uart
        printf("uart initialized\n");
        getchar();

	// initialize lcd
	hd44780_t lcd;
//	hd44780_init(&lcd, LCD_ROWS, LCD_COLS, HD44780_FS_BITS_4, HD44780_FS_LINES_2, HD44780_FS_FONT_5x8,
//	hd44780_init(&lcd, LCD_ROWS, LCD_COLS, HD44780_FS_BITS_4, HD44780_FS_LINES_1, HD44780_FS_FONT_5x10,
//			PIN_RS, PIN_RW, PIN_EN, PIN_NOT_USED, PIN_NOT_USED, PIN_NOT_USED, PIN_NOT_USED, PIN_DATA4, PIN_DATA5, PIN_DATA6, PIN_DATA7);
	hd44780_init(&lcd, LCD_ROWS, LCD_COLS, HD44780_FS_BITS_8, HD44780_FS_LINES_2, HD44780_FS_FONT_5x8,
			PIN_RS, PIN_RW, PIN_EN, PIN_DATA0, PIN_DATA1, PIN_DATA2, PIN_DATA3, PIN_DATA4, PIN_DATA5, PIN_DATA6, PIN_DATA7);
        printf("lcd initialized\n");
        getchar();

	// set display on, cursor on, blink on
	hd44780_disp_control(&lcd, HD44780_DC_DISPLAY_ON | HD44780_DC_CURSOR_ON | HD44780_DC_CURSOR_BLINK_ON);
        printf("lcd on\n");
        getchar();

	// send test string
	const char *test_string = "AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz0123456789!@#$%^&*()-_+=";
	hd44780_entry_mode(&lcd, HD44780_EM_DIRECTION_RIGHT | HD44780_EM_SHIFT_CURSOR);
	hd44780_write_data(&lcd, test_string, strlen(test_string));
        printf("print test\n");
        getchar();

	// display
        printf("display off\n");
	hd44780_display(&lcd, HD44780_DC_DISPLAY_OFF);
        getchar();
        printf("display on\n");
	hd44780_display(&lcd, HD44780_DC_DISPLAY_ON);
        getchar();

	// cursor
        printf("cursor off\n");
	hd44780_cursor(&lcd, HD44780_DC_CURSOR_OFF);
        getchar();
        printf("cursor on\n");
	hd44780_cursor(&lcd, HD44780_DC_CURSOR_ON);
        getchar();

	// cursor blink
        printf("cursor blink off\n");
	hd44780_cursor_blink(&lcd, HD44780_DC_CURSOR_BLINK_OFF);
        getchar();
        printf("cursor blink on\n");
	hd44780_cursor_blink(&lcd, HD44780_DC_CURSOR_BLINK_ON);
        getchar();

	// move cursor
        printf("move cursor\n");
	hd44780_clear(&lcd);
	hd44780_home(&lcd);
	hd44780_write_data(&lcd, test_string, 1);
	hd44780_set_cursor(&lcd, 1, 10);
	hd44780_write_data(&lcd, test_string, 1);
	hd44780_set_cursor(&lcd, 2, 5);
	hd44780_write_data(&lcd, test_string, 1);
	hd44780_set_cursor(&lcd, 3, 15);
	hd44780_write_data(&lcd, test_string, 1);

	printf("end program\n");
	return 0;
	}

