/**
 ******************************************************************************
 * @file    drv_controller.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte da controladora de drivers.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_controller.h"
#include "hal_global.h"
#include "drv_led.h"
#include "drv_relay.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_controller";
static SemaphoreHandle_t led_semphr, relay_semphr;
static ptr_get_drv_t drv_init_vect[drv_end] = {
	drv_led_get_driver,
	drv_relay_get_driver
};
static drv_t *drv_loaded[drv_end];
static uint8_t drv_qtn_loaded;

/* Private functions prototypes ----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

hal_result_t drv_init_controller(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	drv_qtn_loaded = 0;

	led_semphr = xSemaphoreCreateBinary();
	xSemaphoreGive(led_semphr);

	relay_semphr = xSemaphoreCreateBinary();
	xSemaphoreGive(relay_semphr);

	return hal_result_ok;
}

hal_result_t drv_init_drv(uint8_t drv_id, void *parameters)
{
	hal_result_t hal_result = hal_result_fail;

	ESP_LOGI(TAG, "%s", __FUNCTION__);

	if (drv_qtn_loaded < drv_end)
	{
		drv_loaded[drv_qtn_loaded] = drv_init_vect[drv_id]();
		hal_result = drv_loaded[drv_qtn_loaded]->init(drv_id, parameters);
		drv_qtn_loaded++;
	}

	return hal_result;
}

hal_result_t drv_call_drv(uint8_t drv_id, uint8_t drv_func_id, void *parameters)
{
	uint8_t i;
	hal_result_t hal_result = hal_result_fail;

	for (i=0; i<drv_qtn_loaded; i++)
	{
		if (drv_id == drv_loaded[i]->id)
		{
			switch(drv_id)
			{
				case drv_led:
					xSemaphoreTake(led_semphr, portMAX_DELAY);
					break;
				case drv_relay:
					xSemaphoreTake(relay_semphr, portMAX_DELAY);
					break;
			}

			hal_result = drv_loaded[i]->function[drv_func_id](parameters);

			switch(drv_id)
			{
				case drv_led:
					xSemaphoreGive(led_semphr);
					break;
				case drv_relay:
					xSemaphoreGive(relay_semphr);
					break;
			}
		}
	}

	return hal_result;
}

/* Private functions ---------------------------------------------------------*/



