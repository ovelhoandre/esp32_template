/**
 ******************************************************************************
 * @file    drv_i2c.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header de driver i2c.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_I2C_H
#define _DRV_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_i2c_write_byte_id,
	drv_i2c_read_byte_id,
	drv_i2c_write_cmd_id,
	drv_i2c_read_cmd_id,
	drv_i2c_write_block_id,
	drv_i2c_read_block_id,
	drv_i2c_end
};

typedef enum
{
	drv_i2c_num_0 = 0,
	drv_i2c_num_1
} drv_i2c_num_t;

typedef struct
{
	drv_i2c_num_t i2c_num;
	uint8_t scl_io_pin;
	uint8_t sda_io_pin;
	uint32_t clock;
} drv_i2c_init_conf_t;

typedef struct
{
	drv_i2c_num_t i2c_num;
	uint32_t timeout;
	uint8_t slave_addr;
	uint8_t command;
	uint8_t data[255];
	uint8_t len;
} drv_i2c_rw_conf_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_i2c_get_driver(void);

#endif /*_DRV_I2C_H */
