/**
 ******************************************************************************
 * @file    drv_generic.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo header de template de driver.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_GENERIC_H
#define _DRV_GENERIC_H

/* Includes ------------------------------------------------------------------*/
#include "esp_err.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_generic_func1_id,
	drv_generic_func2_id,
	drv_generic_end
};

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_generic_get_driver(void);

#endif /*_DRV_GENERIC_H */
