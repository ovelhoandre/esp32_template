/**
 ******************************************************************************
 * @file    drv_lcd1602.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte do driver lcd1602 com IO expander.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include "drv_controller.h"
#include "hal_types.h"
#include "drv_pcf8574.h"
#include "drv_i2c.h"
#include "drv_lcd1602.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "driver/ledc.h"
#include "driver/dac_oneshot.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

// Delays (microseconds)
#define DELAY_POWER_ON            50000  // wait at least 40us after VCC rises to 2.7V
#define DELAY_INIT_1               4500  // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_2               4500  // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_3                120  // wait at least 100us (fig 24, page 46)

#define DELAY_CLEAR_DISPLAY        2000
#define DELAY_RETURN_HOME          2000

#define DELAY_ENABLE_PULSE_WIDTH      1  // enable pulse must be at least 450ns wide
#define DELAY_ENABLE_PULSE_SETTLE    50  // command requires > 37us to settle (table 6 in datasheet)

// Commands
#define COMMAND_CLEAR_DISPLAY       0x01
#define COMMAND_RETURN_HOME         0x02
#define COMMAND_ENTRY_MODE_SET      0x04
#define COMMAND_DISPLAY_CONTROL     0x08
#define COMMAND_SHIFT               0x10
#define COMMAND_FUNCTION_SET        0x20
#define COMMAND_SET_CGRAM_ADDR      0x40
#define COMMAND_SET_DDRAM_ADDR      0x80

// COMMAND_ENTRY_MODE_SET flags
#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT       0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_DECREMENT       0x00
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON        0x01
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF       0x00

// COMMAND_DISPLAY_CONTROL flags
#define FLAG_DISPLAY_CONTROL_DISPLAY_ON  0x04
#define FLAG_DISPLAY_CONTROL_DISPLAY_OFF 0x00
#define FLAG_DISPLAY_CONTROL_CURSOR_ON   0x02
#define FLAG_DISPLAY_CONTROL_CURSOR_OFF  0x00
#define FLAG_DISPLAY_CONTROL_BLINK_ON    0x01
#define FLAG_DISPLAY_CONTROL_BLINK_OFF   0x00

// COMMAND_SHIFT flags
#define FLAG_SHIFT_MOVE_DISPLAY          0x08
#define FLAG_SHIFT_MOVE_CURSOR           0x00
#define FLAG_SHIFT_MOVE_LEFT             0x04
#define FLAG_SHIFT_MOVE_RIGHT            0x00

// COMMAND_FUNCTION_SET flags
#define FLAG_FUNCTION_SET_MODE_8BIT      0x10
#define FLAG_FUNCTION_SET_MODE_4BIT      0x00
#define FLAG_FUNCTION_SET_LINES_2        0x08
#define FLAG_FUNCTION_SET_LINES_1        0x00
#define FLAG_FUNCTION_SET_DOTS_5X10      0x04
#define FLAG_FUNCTION_SET_DOTS_5X8       0x00

// Control flags
#define FLAG_BACKLIGHT_ON    0b00001000      // backlight enabled (disabled if clear)
#define FLAG_BACKLIGHT_OFF   0b00000000      // backlight disabled
#define FLAG_ENABLE          0b00000100
#define FLAG_READ            0b00000010      // read (write if clear)
#define FLAG_WRITE           0b00000000      // write
#define FLAG_RS_DATA         0b00000001      // data (command if clear)
#define FLAG_RS_COMMAND      0b00000000      // command

#define GPIO_LCD_BLK		  (25)
#define GPIO_LCD_VO			  (26)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_lcd1602";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_lcd1602_end];
static uint8_t _display_control_flags;	// < Currently active display control flags
static uint8_t _entry_mode_flags;		// < Currently active entry mode flags
static dac_oneshot_handle_t chan1_handle;

static uint8_t _char_custom_0[8] = {0x00, 0x0E, 0x11, 0x04, 0x0A, 0x00, 0x04, 0x00}; // Simbolo Wi-Fi
static uint8_t _char_custom_1[8] = {0x00, 0x00, 0x01, 0x02, 0x14, 0x08, 0x00, 0x00}; // Simbolo sinalsinho de check
static uint8_t _char_custom_2[8] = {0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00}; // X

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_lcd1602_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_lcd1602_reset(void *parameters);
hal_result_t _drv_lcd1602_clear(void *parameters);
hal_result_t _drv_lcd1602_home(void *parameters);
hal_result_t _drv_lcd1602_move_cursor(void *parameters);
hal_result_t _drv_lcd1602_write_char(void *parameters);
hal_result_t _drv_lcd1602_write_string(void *parameters);
hal_result_t _drv_lcd1602_set_backlight(void *parameters);
hal_result_t _drv_lcd1602_set_contrast(void *parameters);
hal_result_t _drv_lcd1602_define_char(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_lcd1602_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_lcd1602_init;
	_this_drv.function = _this_function;
	_this_function[drv_lcd1602_reset_id] = _drv_lcd1602_reset;
	_this_function[drv_lcd1602_clear_id] = _drv_lcd1602_clear;
	_this_function[drv_lcd1602_home_id] = _drv_lcd1602_home;
	_this_function[drv_lcd1602_clear_id] = _drv_lcd1602_clear;
	_this_function[drv_lcd1602_move_cursor_id] = _drv_lcd1602_move_cursor;
	_this_function[drv_lcd1602_write_char_id] = _drv_lcd1602_write_char;
	_this_function[drv_lcd1602_write_string_id] = _drv_lcd1602_write_string;
	_this_function[drv_lcd1602_set_backlight_id] = _drv_lcd1602_set_backlight;
	_this_function[drv_lcd1602_set_contrast_id] = _drv_lcd1602_set_contrast;
	_this_function[drv_lcd1602_define_char_id] = _drv_lcd1602_define_char;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

// send data to the I/O Expander
static hal_result_t _write_to_expander(uint8_t data)
{
	drv_pcf8574_port_t port;
	port.raw = data;
    return drv_call_drv(drv_pcf8574, drv_pcf8574_write_id, &port);
}

// clock data from expander to LCD by causing a falling edge on Enable
static hal_result_t _strobe_enable(uint8_t data)
{
	if (_write_to_expander(data | FLAG_ENABLE) != hal_result_ok)
	{
		return hal_result_fail;
	}
    //ets_delay_us(DELAY_ENABLE_PULSE_WIDTH);
	if (_write_to_expander(data & ~FLAG_ENABLE) != hal_result_ok)
	{
		return hal_result_fail;
	}
    //ets_delay_us(DELAY_ENABLE_PULSE_SETTLE);
    return hal_result_ok;
}

// send top nibble to the LCD controller
static hal_result_t _write_top_nibble(uint8_t data)
{
    ESP_LOGD(TAG, "_write_top_nibble 0x%02x", data);
    if (_write_to_expander(data) != hal_result_ok)
    {
    	return hal_result_fail;
    }
    if (_strobe_enable(data) != hal_result_ok)
    {
    	return hal_result_fail;
    }
    return hal_result_ok;
}

// send command or data to controller
static hal_result_t _write(uint8_t value, uint8_t register_select_flag)
{
    ESP_LOGD(TAG, "_write 0x%02x | 0x%02x", value, register_select_flag);
    if (_write_top_nibble((value & 0xf0) | register_select_flag) != hal_result_ok)
    {
    	return hal_result_fail;
    }
    if (_write_top_nibble(((value & 0x0f) << 4) | register_select_flag) != hal_result_ok)
    {
    	return hal_result_fail;
    }
    return hal_result_ok;
}

// send command to controller
static hal_result_t _write_command(uint8_t command)
{
    ESP_LOGD(TAG, "_write_command 0x%02x", command);
    return _write(command, FLAG_RS_COMMAND);
}

// send data to controller
static hal_result_t _write_data(uint8_t data)
{
    ESP_LOGD(TAG, "_write_data 0x%02x", data);
    return _write(data, FLAG_RS_DATA);
}

hal_result_t _drv_lcd1602_init(uint8_t drv_id, void *parameters)
{
	hal_result_t hal_result = hal_result_fail;

	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

    // display on, no cursor, no blinking
    _display_control_flags = FLAG_DISPLAY_CONTROL_DISPLAY_ON | FLAG_DISPLAY_CONTROL_CURSOR_OFF | FLAG_DISPLAY_CONTROL_BLINK_OFF;

	// left-justified left-to-right text
	_entry_mode_flags = FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT | FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF;

	// See page 45/46 of HD44780 data sheet for the initialisation procedure.

	// Wait at least 40ms after power rises above 2.7V before sending commands.
	ets_delay_us(DELAY_POWER_ON);

	hal_result = _drv_lcd1602_reset(parameters);

	drv_lcd1602_data_t lcd1602_data;
	lcd1602_data.str[0] = I2C_LCD1602_CHARACTER_CUSTOM_0;
	_drv_lcd1602_define_char(&lcd1602_data);
	lcd1602_data.str[0] = I2C_LCD1602_CHARACTER_CUSTOM_1;
	_drv_lcd1602_define_char(&lcd1602_data);
	lcd1602_data.str[0] = I2C_LCD1602_CHARACTER_CUSTOM_2;
	_drv_lcd1602_define_char(&lcd1602_data);

    /* DAC oneshot init */
    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &chan1_handle));

    return hal_result;
}

hal_result_t _drv_lcd1602_reset(void *parameters)
{
	hal_result_t first_err = hal_result_ok;
	hal_result_t last_err = hal_result_fail;

    // put Expander into known state - Register Select and Read/Write both low
    if ((last_err = _write_to_expander(0)) != hal_result_ok)
    {
        if (first_err == hal_result_ok) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_to_expander 1 failed: %d", last_err);
    }

    ets_delay_us(1000);

    // select 4-bit mode on LCD controller - see datasheet page 46, figure 24.
    if ((last_err = _write_top_nibble(0x03 << 4)) != hal_result_ok)
    {
        if (first_err == hal_result_ok) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 1 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_1);

    // repeat
    if ((last_err = _write_top_nibble(0x03 << 4)) != hal_result_ok)
    {
        if (first_err == hal_result_ok) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 2 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_2);

    // repeat
    if ((last_err = _write_top_nibble(0x03 << 4)) != hal_result_ok)
    {
        if (first_err == hal_result_ok) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 3 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_3);

    // select 4-bit mode
    if ((last_err = _write_top_nibble(0x02 << 4)) != hal_result_ok)
    {
        if (first_err == hal_result_ok) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 4 failed: %d", last_err);
    }

    // now we can use the command()/write() functions
    if ((last_err = _write_command(COMMAND_FUNCTION_SET | FLAG_FUNCTION_SET_MODE_4BIT | FLAG_FUNCTION_SET_LINES_2 | FLAG_FUNCTION_SET_DOTS_5X8)) != hal_result_ok)
    {
        if (first_err == hal_result_ok)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 1 failed: %d", last_err);
    }

    if ((last_err = _write_command(COMMAND_DISPLAY_CONTROL | _display_control_flags)) != hal_result_ok)
    {
        if (first_err == hal_result_ok)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 2 failed: %d", last_err);
    }

    if ((last_err = _drv_lcd1602_clear(parameters)) != hal_result_ok)
    {
        if (first_err == hal_result_ok)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_clear failed: %d", last_err);
    }

    if ((last_err = _write_command(COMMAND_ENTRY_MODE_SET | _entry_mode_flags)) != hal_result_ok)
    {
        if (first_err == hal_result_ok)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 3 failed: %d", last_err);
    }

    if ((last_err = _drv_lcd1602_home(parameters)) != hal_result_ok)
    {
        if (first_err == hal_result_ok)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_home failed: %d", last_err);
    }

    return first_err;
}

hal_result_t _drv_lcd1602_clear(void *parameters)
{
    hal_result_t err = hal_result_fail;

        err = _write_command(COMMAND_CLEAR_DISPLAY);
        if (err == hal_result_ok)
        {
            ets_delay_us(DELAY_CLEAR_DISPLAY);
        }

    return err;
}

hal_result_t _drv_lcd1602_home(void *parameters)
{
    hal_result_t err = hal_result_fail;

        err = _write_command(COMMAND_RETURN_HOME);
        if (err == hal_result_ok)
        {
            ets_delay_us(DELAY_RETURN_HOME);
        }

    return err;
}

hal_result_t _drv_lcd1602_move_cursor(void *parameters)
{
    hal_result_t err = hal_result_fail;

        const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

        err = _write_command(COMMAND_SET_DDRAM_ADDR | (((drv_lcd1602_data_t *)parameters)->col + row_offsets[((drv_lcd1602_data_t *)parameters)->row]));

    return err;
}

hal_result_t _drv_lcd1602_write_char(void *parameters)
{
    hal_result_t err = hal_result_fail;

        err = _write_data(((drv_lcd1602_data_t *)parameters)->str[0]);

    return err;
}

hal_result_t _drv_lcd1602_write_string(void *parameters)
{
	hal_result_t err = hal_result_ok;

	switch (((drv_lcd1602_data_t *)parameters)->orientation)
	{
		case drv_lcd1602_orientation_left:
			((drv_lcd1602_data_t *)parameters)->col = 0;
			break;
		case drv_lcd1602_orientation_right:
			((drv_lcd1602_data_t *)parameters)->col = 16 - strlen((const char *)((drv_lcd1602_data_t *)parameters)->str);
			break;
		case drv_lcd1602_orientation_center:
			if ((strlen((const char *)((drv_lcd1602_data_t *)parameters)->str)%2) == 0)
			{
				((drv_lcd1602_data_t *)parameters)->col = 8 - (strlen((const char *)((drv_lcd1602_data_t *)parameters)->str)/2);
			}
			else
			{
				((drv_lcd1602_data_t *)parameters)->col = 7 - (strlen((const char *)((drv_lcd1602_data_t *)parameters)->str)/2);
			}
			break;
		case drv_lcd1602_orientation_none:

			break;
	}

	_drv_lcd1602_move_cursor(parameters);

	for(uint8_t i=0; (i<strlen((const char *)((drv_lcd1602_data_t *)parameters)->str)) && (i<16) && (err == hal_result_ok); i++)
	{
		err = _write_data(((drv_lcd1602_data_t *)parameters)->str[i]);
	}

    return err;
}

hal_result_t _drv_lcd1602_set_backlight(void *parameters)
{
	ledc_timer_config_t ledc_timer = {
	     .duty_resolution = LEDC_TIMER_10_BIT,
	     .freq_hz = 1000,
	     .speed_mode = LEDC_HIGH_SPEED_MODE,
	     .timer_num = LEDC_TIMER_0,
	     .clk_cfg = LEDC_AUTO_CLK,
	 };
	 ledc_timer_config(&ledc_timer);

	 ledc_channel_config_t contrast_channel;
	 contrast_channel.channel = LEDC_CHANNEL_0;
	 contrast_channel.duty = 2048 - ((*(uint8_t *)parameters * 512) / 100);
	 contrast_channel.gpio_num = GPIO_LCD_BLK;
	 contrast_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
	 contrast_channel.hpoint = 0;
	 contrast_channel.timer_sel = LEDC_TIMER_0;
	 contrast_channel.intr_type = LEDC_INTR_DISABLE;
	 ledc_channel_config(&contrast_channel);

	 return hal_result_ok;
}

hal_result_t _drv_lcd1602_set_contrast(void *parameters)
{
	if (dac_oneshot_output_voltage(chan1_handle, 255 - ((*(uint8_t *)parameters * 255) / 100)) != ESP_OK)
	{
		return hal_result_fail;
	}

	return hal_result_ok;
}

hal_result_t _drv_lcd1602_define_char(void *parameters)
{
    hal_result_t err = hal_result_fail;

    err = _write_command(COMMAND_SET_CGRAM_ADDR | (((drv_lcd1602_data_t *)parameters)->str[0] << 3));

    for (int i = 0; err == hal_result_ok && i < 8; i++)
    {
    	if (((drv_lcd1602_data_t *)parameters)->str[0] == I2C_LCD1602_CHARACTER_CUSTOM_0)
    	{
    		err = _write_data(_char_custom_0[i]);
    	}
    	else if (((drv_lcd1602_data_t *)parameters)->str[0] == I2C_LCD1602_CHARACTER_CUSTOM_1)
    	{
    		err = _write_data(_char_custom_1[i]);
    	}
    	else if (((drv_lcd1602_data_t *)parameters)->str[0] == I2C_LCD1602_CHARACTER_CUSTOM_2)
    	{
    		err = _write_data(_char_custom_2[i]);
    	}
	}

    return err;
}

