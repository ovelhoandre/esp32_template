/**
 ******************************************************************************
 * @file    hal_global.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte globais ao projeto.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hal_types.h"
#include "hal_global.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static hal_global_data_t global_data;

/* Private functions prototypes ----------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

hal_result_t hal_global_init(void)
{
	memset(&global_data, 0, sizeof(global_data));

#if defined(PRJ_DEBUG)
	esp_log_level_set("*", ESP_LOG_ERROR);
	esp_log_level_set("main", ESP_LOG_INFO);
	esp_log_level_set("drv_controller", ESP_LOG_INFO);
	esp_log_level_set("drv_generic", ESP_LOG_INFO);
	//esp_log_level_set("drv_led", ESP_LOG_INFO);
	//esp_log_level_set("drv_relay", ESP_LOG_INFO);
	esp_log_level_set("drv_i2c", ESP_LOG_INFO);
#else
	esp_log_level_set("*", ESP_LOG_NONE);
#endif
	/* Watchdog init */
	esp_task_wdt_config_t twdt_config = {
	        .timeout_ms = 10000, // 10s
	        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Bitmask of all cores
	        .trigger_panic = true
	};
#if !CONFIG_ESP_TASK_WDT_INIT
	ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
#else
	ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
#endif

	return hal_result_ok;
}

hal_global_data_t *hal_global_get_data(void)
{
	return &global_data;
}

/* Private functions ---------------------------------------------------------*/



