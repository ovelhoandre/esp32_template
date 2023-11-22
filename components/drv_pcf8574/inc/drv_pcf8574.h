/**
 ******************************************************************************
 * @file    drv_pcf8574.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo header do IO Expander.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_PCF8574_H
#define _DRV_PCF8574_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_pcf8574_write_id,
	drv_pcf8574_end
};

typedef union
{
	struct
	{
		uint8_t bit0 : 1;
		uint8_t bit1 : 1;
		uint8_t bit2 : 1;
		uint8_t bit3 : 1;
		uint8_t bit4 : 1;
		uint8_t bit5 : 1;
		uint8_t bit6 : 1;
		uint8_t bit7 : 1;
	} bits;

	uint8_t raw;
} drv_pcf8574_port_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_pcf8574_get_driver(void);

#endif /* _DRV_PCF8574_H */
