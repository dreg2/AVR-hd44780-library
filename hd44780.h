#ifndef HD44780_H_
#define HD44780_H_

#include "pin.h"

// device structure
typedef struct hd44780_s
	{
	uint8_t struct_valid;   // structure valid flag
	uint8_t device_valid;   // device valid flag
	uint8_t rows;           // number of display rows
	uint8_t cols;           // number of display columns
	uint8_t ir_data;        // data read from instruction register

	uint8_t entry_mode;     // saved entry mode command
	uint8_t disp_control;   // saved display control command
	uint8_t cd_shift;       // saved cursor/display shift command
	uint8_t func_set;       // saved function set command
	uint8_t cgram_addr;     // saved cgram address command
	uint8_t ddram_addr;     // saved ddram address command

	pin_t   pin_rs;         // register select pin
	pin_t   pin_rw;         // read/write pin
	pin_t   pin_en;         // enable pin
	pin_t   pin_data[8];    // data pin array
	} hd44780_t;

// prototypes

// low level
uint8_t  hd44780_read(hd44780_t *dev, uint8_t reg_sel);
uint8_t  hd44780_read_byte(hd44780_t *dev, uint8_t reg_sel);

void     hd44780_busy_flag_wait(hd44780_t *dev);

void     hd44780_write(hd44780_t *dev, uint8_t reg_sel, uint8_t data);
void     hd44780_write_byte(hd44780_t *dev, uint8_t reg_sel, uint8_t data_byte);

void     hd44780_command(hd44780_t *dev, uint8_t command, uint8_t options);

void     hd44780_init_struct(hd44780_t *dev, uint8_t rows, uint8_t cols,
                uint8_t pin_rs_ard, uint8_t pin_rw_ard, uint8_t pin_en_ard,
                uint8_t data_0_ard, uint8_t data_1_ard, uint8_t data_2_ard, uint8_t data_3_ard,
                uint8_t data_4_ard, uint8_t data_5_ard, uint8_t data_6_ard, uint8_t data_7_ard);
int8_t   hd44780_init_device(hd44780_t *dev, uint8_t bit_mode, uint8_t lines, uint8_t font);

// high level
int8_t   hd44780_entry_mode_direction(hd44780_t *dev, uint8_t option);
int8_t   hd44780_entry_mode_shift(hd44780_t *dev, uint8_t option);
int8_t   hd44780_display(hd44780_t *dev, uint8_t option);
int8_t   hd44780_cursor(hd44780_t *dev, uint8_t option);
int8_t   hd44780_cursor_blink(hd44780_t *dev, uint8_t option);
int8_t   hd44780_cd_shift(hd44780_t *dev, uint8_t option);
int8_t   hd44780_cd_shift_direction(hd44780_t *dev, uint8_t option);
int8_t   hd44780_cgram_addr(hd44780_t *dev, uint8_t option);
int8_t   hd44780_ddram_addr(hd44780_t *dev, uint8_t option);

int8_t   hd44780_cursor_pos(hd44780_t *dev, uint8_t col, uint8_t row);
void     hd44780_write_data(hd44780_t *dev, const char *data, size_t data_len);


// flag values
#define HD44780_VALID                  0xFF
#define HD44780_INVALID                0xFE

// register select values
#define HD44780_REG_SEL_IR             0x00
#define HD44780_REG_SEL_DR             0x01

// hd44780 instruction register constants
#define HD44780_IR_BUSY_FLAG_MASK      0x80
#define HD44780_IR_ADDR_COUNTER_MASK   0x7F

#define HD44780_CLEAR_DISPLAY_CMD      0x01
#define HD44780_RETURN_HOME_CMD        0x02

#define HD44780_ENTRY_MODE_CMD         0x04
#define HD44780_EM_OPTIONS_MASK        0x03
#define HD44780_EM_DIRECTION_MASK      0x02
#define HD44780_EM_DIRECTION_LEFT      0x00
#define HD44780_EM_DIRECTION_RIGHT     0x02
#define HD44780_EM_SHIFT_MASK          0x01
#define HD44780_EM_SHIFT_CURSOR        0x00
#define HD44780_EM_SHIFT_DISPLAY       0x01

#define HD44780_DISPLAY_CONTROL_CMD    0x08
#define HD44780_DC_OPTIONS_MASK        0x07
#define HD44780_DC_DISPLAY_MASK        0x04
#define HD44780_DC_DISPLAY_OFF         0x00
#define HD44780_DC_DISPLAY_ON          0x04
#define HD44780_DC_CURSOR_MASK         0x02
#define HD44780_DC_CURSOR_OFF          0x00
#define HD44780_DC_CURSOR_ON           0x02
#define HD44780_DC_CURSOR_BLINK_MASK   0x01
#define HD44780_DC_CURSOR_BLINK_OFF    0x00
#define HD44780_DC_CURSOR_BLINK_ON     0x01

#define HD44780_CUR_DISP_SHIFT_CMD     0x10
#define HD44780_CD_OPTIONS_MASK        0x0C
#define HD44780_CD_SHIFT_MASK          0x08
#define HD44780_CD_SHIFT_CURSOR        0x00
#define HD44780_CD_SHIFT_DISPLAY       0x08
#define HD44780_CD_DIRECTION_MASK      0x04
#define HD44780_CD_DIRECTION_LEFT      0x00
#define HD44780_CD_DIRECTION_RIGHT     0x04

#define HD44780_FUNCTION_SET_CMD       0x20
#define HD44780_FS_OPTIONS_MASK        0x1C
#define HD44780_FS_BITS_MASK           0x10
#define HD44780_FS_BITS_4              0x00
#define HD44780_FS_BITS_8              0x10
#define HD44780_FS_LINES_MASK          0x08
#define HD44780_FS_LINES_1             0x00
#define HD44780_FS_LINES_2             0x08
#define HD44780_FS_FONT_MASK           0x04
#define HD44780_FS_FONT_5x8            0x00
#define HD44780_FS_FONT_5x10           0x04

#define HD44780_SET_CGRAM_ADDR_CMD     0x40
#define HD44780_SET_CGRAM_ADDR_MASK    0x3F
#define HD44780_SET_DDRAM_ADDR_CMD     0x80
#define HD44780_SET_DDRAM_ADDR_MASK    0x7F

#endif // HD44780_H_
