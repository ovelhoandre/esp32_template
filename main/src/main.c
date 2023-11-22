/**
 ******************************************************************************
 * @file    main.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo fonte do fw Diponto Multi-Room.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "drv_controller.h"
#include "drv_led.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "main";
static drv_led_state_t _led_state;

/* Private functions prototypes ----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Main
 * @param  None
 * @retval None
 */
void app_main(void)
{
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

    ESP_LOGI(TAG, "Driver controller init");
    DRV_ERROR_CHECK(drv_init_controller());

	ESP_LOGI(TAG, "LED driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_led, NULL));

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

	while (1)
	{
		/* Chama funcao get state do led */
		DRV_ERROR_CHECK(drv_call_drv(drv_led, drv_led_get_state_id, &_led_state));

		if (_led_state == drv_led_state_off)
		{
			_led_state = drv_led_state_on;
		}
		else
		{
			_led_state = drv_led_state_off;
		}

		/* Chama funcao set state do led */
		DRV_ERROR_CHECK(drv_call_drv(drv_led, drv_led_set_state_id, &_led_state));

		vTaskDelay(1000 / portTICK_PERIOD_MS);

		esp_task_wdt_reset();
	}
}


