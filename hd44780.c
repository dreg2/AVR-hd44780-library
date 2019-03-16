#include <stdarg.h>
#include <stdio.h>
#include <util/delay.h>

#include "pin.h"
#include "hd44780.h"


//----------------------------------------------------------------------------------------------------
// read data from hd44780
//----------------------------------------------------------------------------------------------------
uint8_t hd44780_read(hd44780_t *dev, uint8_t reg_sel)
	{
	// set register select pin
	if (reg_sel == HD44780_PIN_RS_IR)
		pin_state_set(&dev->pin_rs, PIN_OUT_LOW);  // instruction register
	else
		pin_state_set(&dev->pin_rs, PIN_OUT_HIGH); // data register

	// set R/W pin high for read
	pin_state_set(&dev->pin_rw, PIN_OUT_HIGH);

	// setup for 4 or 8 pin read
	uint8_t pin_count = 8;
	uint8_t pin_offset = 0;
	if ((dev->func_set & HD44780_FS_BITS_MASK) == HD44780_FS_BITS_4)
		{
		pin_count  = 4;
		pin_offset = 4;
		}

	// set data pins for input
	for (uint8_t i = 0; i < pin_count; i++)
		pin_state_set(&dev->pin_data[i+pin_offset], PIN_IN_HIGHZ);

	// activate enable pin
	pin_state_set(&dev->pin_en, PIN_OUT_HIGH);

	// read data
	uint8_t data = 0;
	for (uint8_t i = 0; i < pin_count; i++)
		data |= (uint8_t)(pin_in(&dev->pin_data[i+pin_offset]) << i);

	// deactivate enable
	pin_state_set(&dev->pin_en, PIN_OUT_LOW);

//	_delay_us(300);
	return data;
	}

//----------------------------------------------------------------------------------------------------
// read bytes from hd44780
//----------------------------------------------------------------------------------------------------
uint8_t hd44780_read_byte(hd44780_t *dev, uint8_t reg_sel)
	{
	uint8_t data_byte = 0;

	if ((dev->func_set & HD44780_FS_BITS_MASK) == HD44780_FS_BITS_4)
		{
		// 4-bit read
		data_byte = (uint8_t)((hd44780_read(dev, reg_sel) << 4) | hd44780_read(dev, reg_sel));
		}
	else
		{
		// 8-bit read
		data_byte = hd44780_read(dev, reg_sel);
		}

	return data_byte;
	}

//----------------------------------------------------------------------------------------------------
// wait for busy flag low
//----------------------------------------------------------------------------------------------------
void hd44780_busy_flag_wait(hd44780_t *dev)
	{
//printf("hd44780_busy_flag_wait: ");
	int count = 0;
	while ((dev->last_addr = hd44780_read_byte(dev, HD44780_PIN_RS_IR)) & HD44780_IR_BUSY_FLAG_MASK)
		{
		count++;
		}
//printf("%d  0x%02hx\n", count, dev->last_addr);
	}

//----------------------------------------------------------------------------------------------------
// write data to hd44780
//----------------------------------------------------------------------------------------------------
void hd44780_write(hd44780_t *dev, uint8_t reg_sel, uint8_t data)
	{
//printf("    hd44780_write: 0x%02hx\n", data);
	// set register select pin
	if (reg_sel == HD44780_PIN_RS_IR)
		pin_state_set(&dev->pin_rs, PIN_OUT_LOW);  // instruction register
	else
		pin_state_set(&dev->pin_rs, PIN_OUT_HIGH); // data register

	// set R/W pin low for write
	pin_state_set(&dev->pin_rw, PIN_OUT_LOW);

	// setup for 4 or 8 pin write
	uint8_t pin_count = 8;
	uint8_t pin_offset = 0;
	if ((dev->func_set & HD44780_FS_BITS_MASK) == HD44780_FS_BITS_4)
		{
		pin_count  = 4;
		pin_offset = 4;
		}

	// set data pins
	for (uint8_t i = 0; i < pin_count; i++)
		{
		if (data & (1 << i))
			pin_state_set(&dev->pin_data[i+pin_offset], PIN_OUT_HIGH);
		else
			pin_state_set(&dev->pin_data[i+pin_offset], PIN_OUT_LOW);
		}

	// activate enable
	pin_state_set(&dev->pin_en, PIN_OUT_HIGH);

	// delay for enable pulse width
//	_delay_us(0.230); // PWeh = 230 nanosec pulse width

	// deactivate enable
	pin_state_set(&dev->pin_en, PIN_OUT_LOW);

//	_delay_us(300);
	}

//----------------------------------------------------------------------------------------------------
// write byte to hd44780
//----------------------------------------------------------------------------------------------------
void hd44780_write_byte(hd44780_t *dev, uint8_t reg_sel, uint8_t data_byte)
	{
printf("hd44780_write_byte: 0x%02hx\n", data_byte);
	// write byte
	if ((dev->func_set & HD44780_FS_BITS_MASK) == HD44780_FS_BITS_4)
		{
		// 4-bit write
		hd44780_write(dev, reg_sel, data_byte >> 4);
		hd44780_write(dev, reg_sel, data_byte);
		}
	else
		{
		// 8-bit write
		hd44780_write(dev, reg_sel, data_byte);
		}

	hd44780_busy_flag_wait(dev);
	}

//----------------------------------------------------------------------------------------------------
// set function set
//----------------------------------------------------------------------------------------------------
void hd44780_func_set(hd44780_t *dev, uint8_t options)
	{
	dev->func_set = (uint8_t)(HD44780_FUNCTION_SET_CMD | (options & HD44780_FS_OPTIONS_MASK));
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->func_set);
	}

//----------------------------------------------------------------------------------------------------
// set entry mode
//----------------------------------------------------------------------------------------------------
void hd44780_entry_mode(hd44780_t *dev, uint8_t options)
	{
	dev->entry_mode = (uint8_t)(HD44780_ENTRY_MODE_CMD | (options & HD44780_EM_OPTIONS_MASK));
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->entry_mode);
	}

//----------------------------------------------------------------------------------------------------
// set display control
//----------------------------------------------------------------------------------------------------
void hd44780_disp_control(hd44780_t *dev, uint8_t options)
	{
	dev->disp_control = (uint8_t)(HD44780_DISPLAY_CONTROL_CMD | (options & HD44780_DC_OPTIONS_MASK));
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->disp_control);
	}

//----------------------------------------------------------------------------------------------------
// set cursor/display shift
//----------------------------------------------------------------------------------------------------
void hd44780_cur_disp_shift(hd44780_t *dev, uint8_t options)
	{
	dev->cur_disp_shift = (uint8_t)(HD44780_CUR_DISP_SHIFT_CMD | (options & HD44780_CDS_OPTIONS_MASK));
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->cur_disp_shift);
	}

//----------------------------------------------------------------------------------------------------
// clear display
//----------------------------------------------------------------------------------------------------
void hd44780_clear(hd44780_t *dev)
	{
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, HD44780_CLEAR_DISPLAY_CMD);
	}

//----------------------------------------------------------------------------------------------------
// set cursor to home
//----------------------------------------------------------------------------------------------------
void hd44780_home(hd44780_t *dev)
	{
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, HD44780_RETURN_HOME_CMD);
	}

//----------------------------------------------------------------------------------------------------
// initialize device structure and device
//----------------------------------------------------------------------------------------------------
void hd44780_init(hd44780_t *dev, uint8_t rows, uint8_t cols, uint8_t bit_mode, uint8_t lines, uint8_t font,
		uint8_t pin_rs_ard, uint8_t pin_rw_ard, uint8_t pin_en_ard,
                uint8_t data_0_ard, uint8_t data_1_ard, uint8_t data_2_ard, uint8_t data_3_ard,
                uint8_t data_4_ard, uint8_t data_5_ard, uint8_t data_6_ard, uint8_t data_7_ard)
	{
	// structure initialization

	// intialize control fields
	dev->valid_flag = HD44780_INVALID;
	dev->rows = rows;
	dev->cols = cols;

	// initialize pin fields
	pin_init_ard(&dev->pin_rs, pin_rs_ard);
	pin_init_ard(&dev->pin_rw, pin_rw_ard);
	pin_init_ard(&dev->pin_en, pin_en_ard);
	pin_init_ard(&dev->pin_data[0], data_0_ard);
	pin_init_ard(&dev->pin_data[1], data_1_ard);
	pin_init_ard(&dev->pin_data[2], data_2_ard);
	pin_init_ard(&dev->pin_data[3], data_3_ard);
	pin_init_ard(&dev->pin_data[4], data_4_ard);
	pin_init_ard(&dev->pin_data[5], data_5_ard);
	pin_init_ard(&dev->pin_data[6], data_6_ard);
	pin_init_ard(&dev->pin_data[7], data_7_ard);

	// flag structure as initialized
	dev->valid_flag = HD44780_VALID;


	// device initialization

	// Configure control pins as output and set low
	pin_state_set(&dev->pin_rs, PIN_OUT_LOW); // rs = IR
	pin_state_set(&dev->pin_rw, PIN_OUT_LOW); // rw = write
	pin_state_set(&dev->pin_en, PIN_OUT_LOW); // en = inactive

	// hd44780 initialization routine (4-bit mode)
//printf("hd44780_init: start init\n");
//	_delay_ms(15);
	hd44780_write(dev, HD44780_PIN_RS_IR, 0x03);
//	_delay_ms(4.1);
	hd44780_write(dev, HD44780_PIN_RS_IR, 0x03);
//	_delay_ms(4.1);
	hd44780_write(dev, HD44780_PIN_RS_IR, 0x03);
//	_delay_ms(4.1);
	hd44780_write(dev, HD44780_PIN_RS_IR, 0x02);
//printf("hd44780_init: init done\n");

	// send function set command (can only be done during initialization)
	hd44780_func_set(dev, bit_mode | lines | font);

	// send display control command
	hd44780_disp_control(dev, HD44780_DC_DISPLAY_OFF | HD44780_DC_CURSOR_OFF | HD44780_DC_CURSOR_BLINK_OFF);

	// send clear display command
	hd44780_clear(dev);

	// send entry mode command
	hd44780_entry_mode(dev, HD44780_EM_DIRECTION_LEFT | HD44780_EM_SHIFT_CURSOR);
	}

//----------------------------------------------------------------------------------------------------
// position cursor
//----------------------------------------------------------------------------------------------------
void hd44780_set_cursor(hd44780_t *dev, uint8_t col, uint8_t row)
	{
	static uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row > (HD44780_ROWS_MAX-1))
		row = (HD44780_ROWS_MAX-1);

	hd44780_write_byte(dev, HD44780_PIN_RS_IR, (uint8_t)(HD44780_SET_DDRAM_ADDR_CMD | (col + offsets[row])));
	}

//----------------------------------------------------------------------------------------------------
// write string
//----------------------------------------------------------------------------------------------------
void hd44780_write_string(hd44780_t *dev, const char *data, size_t data_len)
	{
	for (size_t i = 0; i < data_len; i++)
		{
		hd44780_write_byte(dev, HD44780_PIN_RS_DR, (uint8_t)data[i]);
		}
	}

//----------------------------------------------------------------------------------------------------
// entry direction
//----------------------------------------------------------------------------------------------------
int8_t hd44780_entry_mode_direction(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_EM_DIRECTION_LEFT:
			dev->entry_mode &= (uint8_t)~(HD44780_EM_DIRECTION_MASK);
			break;

		case HD44780_EM_DIRECTION_RIGHT:
			dev->entry_mode |= HD44780_EM_DIRECTION_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->entry_mode);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// entry mode shift
//----------------------------------------------------------------------------------------------------
int8_t hd44780_entry_mode_shift(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_EM_SHIFT_CURSOR:
			dev->entry_mode &= (uint8_t)~(HD44780_EM_SHIFT_MASK);
			break;

		case HD44780_EM_SHIFT_DISPLAY:
			dev->entry_mode |= HD44780_EM_SHIFT_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->entry_mode);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// display 
//----------------------------------------------------------------------------------------------------
int8_t hd44780_display(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_DC_DISPLAY_OFF:
			dev->disp_control &= (uint8_t)~(HD44780_DC_DISPLAY_MASK);
			break;

		case HD44780_DC_DISPLAY_ON:
			dev->disp_control |= HD44780_DC_DISPLAY_ON;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// cursor 
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cursor(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_DC_CURSOR_OFF:
			dev->disp_control &= (uint8_t)~(HD44780_DC_CURSOR_MASK);
			break;

		case HD44780_DC_CURSOR_ON:
			dev->disp_control |= HD44780_DC_CURSOR_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// cursor blink
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cursor_blink(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_DC_CURSOR_BLINK_OFF:
			dev->disp_control &= (uint8_t)~(HD44780_DC_CURSOR_BLINK_MASK);
			break;

		case HD44780_DC_CURSOR_BLINK_ON:
			dev->disp_control |= HD44780_DC_CURSOR_BLINK_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// cursor/display shift
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cd_shift(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_CDS_SHIFT_CURSOR:
			dev->cur_disp_shift &= (uint8_t)~(HD44780_CDS_SHIFT_MASK);
			break;

		case HD44780_CDS_SHIFT_DISPLAY:
			dev->cur_disp_shift |= HD44780_CDS_SHIFT_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->cur_disp_shift);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// cursor/display direction
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cd_direction(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_CDS_DIRECTION_LEFT:
			dev->cur_disp_shift &= (uint8_t)~(HD44780_CDS_DIRECTION_MASK);
			break;

		case HD44780_CDS_DIRECTION_RIGHT:
			dev->cur_disp_shift |= HD44780_CDS_DIRECTION_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_PIN_RS_IR, dev->cur_disp_shift);
	return 0;
	}
