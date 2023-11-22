/**
 ******************************************************************************
 * @file    drv_controller.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header da controladora de drivers.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_CONTROLLER_H
#define _DRV_CONTROLLER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "hal_types.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef hal_result_t (*ptr_init_drv_t)(uint8_t drv_id, void *parameters);
typedef hal_result_t (*ptr_func_drv_t)(void *parameters);

typedef struct
{
	uint8_t id;
	ptr_init_drv_t init;
	ptr_func_drv_t *function;
}drv_t;

typedef drv_t *(*ptr_get_drv_t)(void);

enum
{
	drv_led,
	drv_relay,
	drv_i2c,
	drv_keyboard,
	drv_pcf8574,
	drv_end
};

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
hal_result_t drv_init_controller(void);
hal_result_t drv_init_drv(uint8_t drv_id, void *parameters);
hal_result_t drv_call_drv(uint8_t drv_id, uint8_t drv_func_id, void *parameters);

#endif /*_DRV_CONTROLLER_H */
