/**
 ******************************************************************************
 * @file    drv_led.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo header led.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_LED_H
#define _DRV_LED_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_led_set_state_id,
	drv_led_get_state_id,
	drv_led_end
};

typedef enum
{
	drv_led_state_on = 0,
	drv_led_state_off
} drv_led_state_t;

/* Exported macro ------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_led_get_driver(void);

#endif /*_DRV_LED_H */
