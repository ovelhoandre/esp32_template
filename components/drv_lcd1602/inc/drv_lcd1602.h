
/**
 ******************************************************************************
 * @file    drv_lcd1602.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header do driver lcd1602 com IO expander.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _I2C_LCD1602_H
#define _I2C_LCD1602_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
// Special characters for ROM Code A00
#define I2C_LCD1602_CHARACTER_CUSTOM_0     0b000   ///< User-defined custom symbol in index 0
#define I2C_LCD1602_CHARACTER_CUSTOM_1     0b001   ///< User-defined custom symbol in index 1
#define I2C_LCD1602_CHARACTER_CUSTOM_2     0b010   ///< User-defined custom symbol in index 2
#define I2C_LCD1602_CHARACTER_CUSTOM_3     0b011   ///< User-defined custom symbol in index 3
#define I2C_LCD1602_CHARACTER_CUSTOM_4     0b100   ///< User-defined custom symbol in index 4
#define I2C_LCD1602_CHARACTER_CUSTOM_5     0b101   ///< User-defined custom symbol in index 5
#define I2C_LCD1602_CHARACTER_CUSTOM_6     0b110   ///< User-defined custom symbol in index 6
#define I2C_LCD1602_CHARACTER_CUSTOM_7     0b111   ///< User-defined custom symbol in index 7

/* Exported types ------------------------------------------------------------*/
enum
{
	drv_lcd1602_reset_id,
	drv_lcd1602_clear_id,
	drv_lcd1602_home_id,
	drv_lcd1602_move_cursor_id,
	drv_lcd1602_write_char_id,
	drv_lcd1602_write_string_id,
	drv_lcd1602_set_backlight_id,
	drv_lcd1602_set_contrast_id,
	drv_lcd1602_define_char_id,
	drv_lcd1602_end
};

typedef enum
{
	drv_lcd1602_orientation_left = 0,
	drv_lcd1602_orientation_right,
	drv_lcd1602_orientation_center,
	drv_lcd1602_orientation_none
} drv_lcd1602_orientation_t;

typedef struct
{
	uint8_t row;
	uint8_t col;
	drv_lcd1602_orientation_t orientation;
	uint8_t str[20];
} drv_lcd1602_data_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_lcd1602_get_driver(void);

#endif /*_I2C_LCD1602_H */
