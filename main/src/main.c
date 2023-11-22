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
#include "hal_global.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "drv_controller.h"
#include "drv_led.h"
#include "drv_relay.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "main";
static drv_led_state_t _led_state;
drv_relay_conf_t relay_conf;

/* Private functions prototypes ----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Main
 * @param  None
 * @retval None
 */
void app_main(void)
{
#if defined(PRJ_DEBUG)
	esp_log_level_set("*", ESP_LOG_ERROR);
	esp_log_level_set("main", ESP_LOG_INFO);
	esp_log_level_set("drv_controller", ESP_LOG_INFO);
	esp_log_level_set("drv_generic", ESP_LOG_INFO);
	//esp_log_level_set("drv_led", ESP_LOG_INFO);
	//esp_log_level_set("drv_relay", ESP_LOG_INFO);
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

    ESP_LOGI(TAG, "Driver controller init");
    DRV_ERROR_CHECK(drv_init_controller());

	ESP_LOGI(TAG, "LED driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_led, NULL));

	ESP_LOGI(TAG, "Relay driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_relay, NULL));

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

	while (1)
	{
		/* Teste driver led */
		DRV_ERROR_CHECK(drv_call_drv(drv_led, drv_led_get_state_id, &_led_state));
		if (_led_state == drv_led_state_off)
		{
			_led_state = drv_led_state_on;
		}
		else
		{
			_led_state = drv_led_state_off;
		}
		DRV_ERROR_CHECK(drv_call_drv(drv_led, drv_led_set_state_id, &_led_state));

		/* Teste driver relay */
		relay_conf.relay_num = drv_relay_num_0;
		DRV_ERROR_CHECK(drv_call_drv(drv_relay, drv_relay_get_state_id, &relay_conf));
		if (relay_conf.relay_state == drv_relay_state_off)
		{
			relay_conf.relay_state = drv_relay_state_on;
		}
		else
		{
			relay_conf.relay_state = drv_relay_state_off;
		}
		DRV_ERROR_CHECK(drv_call_drv(drv_relay, drv_relay_set_state_id, &relay_conf));

		relay_conf.relay_num = drv_relay_num_1;
		DRV_ERROR_CHECK(drv_call_drv(drv_relay, drv_relay_get_state_id, &relay_conf));
		if (relay_conf.relay_state == drv_relay_state_off)
		{
			relay_conf.relay_state = drv_relay_state_on;
		}
		else
		{
			relay_conf.relay_state = drv_relay_state_off;
		}
		DRV_ERROR_CHECK(drv_call_drv(drv_relay, drv_relay_set_state_id, &relay_conf));

		vTaskDelay(1000 / portTICK_PERIOD_MS);

		esp_task_wdt_reset();
	}
}


