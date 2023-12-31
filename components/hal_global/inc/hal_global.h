/**
 ******************************************************************************
 * @file    hal_global.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header com declaracoes globais ao projeto.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_GLOBAL_H
#define _HAL_GLOBAL_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_relay.h"
#include "drv_i2c.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	/* Drivers */
	drv_i2c_init_conf_t i2c_init_conf;
	drv_i2c_rw_conf_t pcf8574_rw_conf;
	drv_i2c_rw_conf_t rv8803_rw_conf;
} hal_global_data_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
hal_result_t hal_global_init(void);
hal_global_data_t *hal_global_get_data(void);

#endif /*_HAL_GLOBAL_H */
