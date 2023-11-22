/**
 ******************************************************************************
 * @file    drv_relay.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo header relay.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_RELAY_H
#define _DRV_RELAY_H

/* Includes ------------------------------------------------------------------*/
#include "hal_global.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
	drv_relay_set_state_id,
	drv_relay_get_state_id,
	drv_relay_end
};

typedef enum
{
	drv_relay_state_on = 0,
	drv_relay_state_off
} drv_relay_state_t;

typedef enum
{
	drv_relay_num_0 = 0,
	drv_relay_num_1
} drv_relay_num_t;

typedef struct
{
	drv_relay_num_t relay_num;
	drv_relay_state_t relay_state;
} drv_relay_conf_t;

/* Exported macro ------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_relay_get_driver(void);

#endif /*_DRV_RELAY_H */
