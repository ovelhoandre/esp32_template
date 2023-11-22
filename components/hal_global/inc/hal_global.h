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
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
	hal_result_fail = 0,
	hal_result_ok
} hal_result_t;

/* Exported macro ------------------------------------------------------------*/
#define DRV_ERROR_CHECK(x) do {                                                         \
		hal_result_t hal_result = (x);                                                  \
        if (unlikely(hal_result != hal_result_ok)) {                                    \
            _esp_error_check_failed(hal_result, __FILE__, __LINE__, __ASSERT_FUNC, #x); \
        }                                                                               \
    } while(0)

/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#endif /*_HAL_GLOBAL_H */
