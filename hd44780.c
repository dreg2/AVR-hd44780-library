#include <stdarg.h>
#include <stdio.h>
#include <util/delay.h>

#include "common.h"
#include "pin.h"
#include "hd44780.h"


//----------------------------------------------------------------------------------------------------
// read from hd44780
//----------------------------------------------------------------------------------------------------
uint8_t hd44780_read(hd44780_t *dev, uint8_t reg_sel)
	{
	// check for intialized device
	if (dev->device_valid != HD44780_VALID)
		return 0;

	// set register select pin
	if (reg_sel == HD44780_REG_SEL_IR)
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
		pin_state_set(&dev->pin_data[pin_offset + i], PIN_IN_HIGHZ);

	// activate enable pin
	pin_state_set(&dev->pin_en, PIN_OUT_HIGH);

	// read data
	uint8_t data = 0;
	for (uint8_t i = 0; i < pin_count; i++)
		data |= (uint8_t)(pin_in(&dev->pin_data[pin_offset + i]) << i);

	// deactivate enable
	pin_state_set(&dev->pin_en, PIN_OUT_LOW);

	return data;
	}

//----------------------------------------------------------------------------------------------------
// read byte from hd44780
//----------------------------------------------------------------------------------------------------
uint8_t hd44780_read_byte(hd44780_t *dev, uint8_t reg_sel)
	{
	uint8_t data_byte = 0;

	// read byte
	if ((dev->func_set & HD44780_FS_BITS_MASK) == HD44780_FS_BITS_4)
		// 4-bit read
		data_byte = (uint8_t)((hd44780_read(dev, reg_sel) << 4) | hd44780_read(dev, reg_sel));
	else
		// 8-bit read
		data_byte = hd44780_read(dev, reg_sel);

	return data_byte;
	}

//----------------------------------------------------------------------------------------------------
// wait for busy flag low
//----------------------------------------------------------------------------------------------------
void hd44780_busy_flag_wait(hd44780_t *dev)
	{
	while ((dev->ir_data = hd44780_read_byte(dev, HD44780_REG_SEL_IR)) & HD44780_IR_BUSY_FLAG_MASK)
		;
	}

//----------------------------------------------------------------------------------------------------
// write to hd44780
//----------------------------------------------------------------------------------------------------
void hd44780_write(hd44780_t *dev, uint8_t reg_sel, uint8_t data)
	{
	// check for intialized device
	if (dev->device_valid != HD44780_VALID)
		return;

	// set register select pin
	if (reg_sel == HD44780_REG_SEL_IR)
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
			pin_state_set(&dev->pin_data[pin_offset + i], PIN_OUT_HIGH);
		else
			pin_state_set(&dev->pin_data[pin_offset + i], PIN_OUT_LOW);
		}

	// activate enable
	pin_state_set(&dev->pin_en, PIN_OUT_HIGH);

	// deactivate enable
	pin_state_set(&dev->pin_en, PIN_OUT_LOW);
	}

//----------------------------------------------------------------------------------------------------
// write byte to hd44780
//----------------------------------------------------------------------------------------------------
void hd44780_write_byte(hd44780_t *dev, uint8_t reg_sel, uint8_t data_byte)
	{
debug_printf("hd44780_write_byte: %c 0x%02hx\n", (reg_sel == HD44780_REG_SEL_IR ? 'I' : 'D'), data_byte);
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
// initialize device structure
//----------------------------------------------------------------------------------------------------
void hd44780_init_struct(hd44780_t *dev, uint8_t rows, uint8_t cols, uint8_t pin_rs_ard, uint8_t pin_rw_ard, uint8_t pin_en_ard,
		uint8_t data_0_ard, uint8_t data_1_ard, uint8_t data_2_ard, uint8_t data_3_ard,
		uint8_t data_4_ard, uint8_t data_5_ard, uint8_t data_6_ard, uint8_t data_7_ard)
	{
	// intialize control fields
	dev->struct_valid = HD44780_INVALID;
	dev->device_valid = HD44780_INVALID;
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

	// initialize command fields
	dev->entry_mode   = HD44780_ENTRY_MODE_CMD;
	dev->disp_control = HD44780_DISPLAY_CONTROL_CMD;
	dev->cd_shift     = HD44780_CUR_DISP_SHIFT_CMD;
	dev->func_set     = HD44780_FUNCTION_SET_CMD;
	dev->cgram_addr   = HD44780_SET_CGRAM_ADDR_CMD;
	dev->ddram_addr   = HD44780_SET_DDRAM_ADDR_CMD;

	// mark structure as initialized
	dev->struct_valid = HD44780_VALID;
	}

//----------------------------------------------------------------------------------------------------
// initialize device
//----------------------------------------------------------------------------------------------------
void hd44780_init_device(hd44780_t *dev, uint8_t bit_mode, uint8_t lines, uint8_t font)
	{
	// mark device invalid until initialized
	dev->device_valid = HD44780_INVALID;

	// check for initialized structure
	if (dev->struct_valid != HD44780_VALID)
		return;

	// mark device as valid
	dev->device_valid = HD44780_VALID;

	// Configure control pins as output and set low
	pin_state_set(&dev->pin_rs, PIN_OUT_LOW); // rs = IR
	pin_state_set(&dev->pin_rw, PIN_OUT_LOW); // rw = write
	pin_state_set(&dev->pin_en, PIN_OUT_LOW); // en = inactive

	// configure data pins for output and set low
	for (uint8_t i = 0; i < 8; i++)
		pin_state_set(&dev->pin_data[i], PIN_OUT_LOW);

	// initialize device
	if (bit_mode == HD44780_FS_BITS_4)
		{
		// hd44780 initialization routine (4-bit mode)
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x03);
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x03);
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x03);
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x02);
		}
	else
		{
		// hd44780 initialization routine (8-bit mode)
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x30);
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x30);
		hd44780_write(dev, HD44780_REG_SEL_IR, 0x30);
		}

	// write function set command (can only be done during initialization)
	dev->func_set = (uint8_t)(HD44780_FUNCTION_SET_CMD | bit_mode | lines | font);
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->func_set);
	}

//----------------------------------------------------------------------------------------------------
// write command
//----------------------------------------------------------------------------------------------------
void hd44780_command(hd44780_t *dev, uint8_t command, uint8_t options)
	{
	uint8_t command_byte = 0x00;

	// setup command byte
	switch (command)
		{
		case HD44780_CLEAR_DISPLAY_CMD:
			command_byte = HD44780_CLEAR_DISPLAY_CMD;
			break;

		case HD44780_RETURN_HOME_CMD:
			command_byte = HD44780_RETURN_HOME_CMD;
			break;

		case HD44780_ENTRY_MODE_CMD:
			command_byte = (uint8_t)(HD44780_ENTRY_MODE_CMD | (options & HD44780_EM_OPTIONS_MASK));
			dev->entry_mode = command_byte;
			break;

		case HD44780_DISPLAY_CONTROL_CMD:
			command_byte = (uint8_t)(HD44780_DISPLAY_CONTROL_CMD | (options & HD44780_DC_OPTIONS_MASK));
			dev->disp_control = command_byte;
			break;

		case HD44780_CUR_DISP_SHIFT_CMD:
			command_byte = (uint8_t)(HD44780_CUR_DISP_SHIFT_CMD | (options & HD44780_CD_OPTIONS_MASK));
			dev->cd_shift = command_byte;
			break;

		case HD44780_FUNCTION_SET_CMD:
			command_byte = (uint8_t)(HD44780_FUNCTION_SET_CMD | (options & HD44780_FS_OPTIONS_MASK));
			dev->func_set = command_byte;
			break;

		case HD44780_SET_CGRAM_ADDR_CMD:
			command_byte = (uint8_t)(HD44780_SET_CGRAM_ADDR_CMD | (options & HD44780_SET_CGRAM_ADDR_MASK));
			dev->cgram_addr = command_byte;
			break;

		case HD44780_SET_DDRAM_ADDR_CMD:
			command_byte = (uint8_t)(HD44780_SET_DDRAM_ADDR_CMD | (options & HD44780_SET_DDRAM_ADDR_MASK));
			dev->ddram_addr = command_byte;
			break;

		default:
			return;
		}

	// write command byte
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, command_byte);
	}

//----------------------------------------------------------------------------------------------------
// set entry mode direction left/right
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
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->entry_mode);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set entry mode display/cursor shift
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
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->entry_mode);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set display on/off
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
			dev->disp_control |= HD44780_DC_DISPLAY_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set cursor on/off
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
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set cursor blink on/off
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
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->disp_control);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set shift cursor/display
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cd_shift_cd(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_CD_SHIFT_CURSOR:
			dev->cd_shift &= (uint8_t)~(HD44780_CD_SHIFT_MASK);
			break;

		case HD44780_CD_SHIFT_DISPLAY:
			dev->cd_shift |= HD44780_CD_SHIFT_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->cd_shift);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set cursor/display direction
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cd_shift_direction(hd44780_t *dev, uint8_t option)
	{
	// set option
	switch (option)
		{
		case HD44780_CD_DIRECTION_LEFT:
			dev->cd_shift &= (uint8_t)~(HD44780_CD_DIRECTION_MASK);
			break;

		case HD44780_CD_DIRECTION_RIGHT:
			dev->cd_shift |= HD44780_CD_DIRECTION_MASK;
			break;

		default:
			return -1;
		}

	// send command
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->cd_shift);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set cgram address
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cgram_addr(hd44780_t *dev, uint8_t option)
	{
	dev->cgram_addr = (uint8_t)(HD44780_SET_CGRAM_ADDR_CMD | (option & HD44780_SET_CGRAM_ADDR_MASK));

	// send command
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->cgram_addr);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set ddram address
//----------------------------------------------------------------------------------------------------
int8_t hd44780_ddram_addr(hd44780_t *dev, uint8_t option)
	{
	dev->ddram_addr = (uint8_t)(HD44780_SET_DDRAM_ADDR_CMD | (option & HD44780_SET_DDRAM_ADDR_MASK));

	// send command
	hd44780_write_byte(dev, HD44780_REG_SEL_IR, dev->ddram_addr);
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// set cursor position
//----------------------------------------------------------------------------------------------------
int8_t hd44780_cursor_pos(hd44780_t *dev, uint8_t row, uint8_t column)
	{
	static uint8_t row_offset[] = {0x00, 0x40, 0x14, 0x54};

	// validate arguments
	if (row > (dev->rows - 1))
		return -1;
	if (column > (dev->cols - 1))
		return -1;

	hd44780_ddram_addr(dev, (uint8_t)(row_offset[row] + column));
	return 0;
	}

//----------------------------------------------------------------------------------------------------
// write data
//----------------------------------------------------------------------------------------------------
void hd44780_write_data(hd44780_t *dev, const char *data, size_t data_len)
	{
	for (size_t i = 0; i < data_len; i++)
		{
		hd44780_write_byte(dev, HD44780_REG_SEL_DR, (uint8_t)data[i]);
		}
	}
