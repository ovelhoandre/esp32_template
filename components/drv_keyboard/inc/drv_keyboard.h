/**
 ******************************************************************************
 * @file    drv_keyboard.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header do teclado matricial.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_KEYBOARD_H
#define _DRV_KEYBOARD_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_keyboard_get_id,
	drv_keyboard_end
};

typedef union
{
	struct
	{
		uint8_t k0 : 1;
		uint8_t k1 : 1;
		uint8_t k2 : 1;
		uint8_t k3 : 1;
		uint8_t k4 : 1;
		uint8_t k5 : 1;
		uint8_t k6 : 1;
		uint8_t k7 : 1;
		uint8_t k8 : 1;
		uint8_t k9 : 1;
		uint8_t kmenu : 1;
		uint8_t kdel : 1;
		uint8_t kup : 1;
		uint8_t kdown : 1;
		uint8_t kok : 1;
		uint8_t kesc : 1;
		uint16_t value;
	} key;

	uint32_t raw;
} drv_keyboard_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_keyboard_get_driver(void);

#endif /*_DRV_KEYBOARD_H */
