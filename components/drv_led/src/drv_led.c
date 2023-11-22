/**
 ******************************************************************************
 * @file    drv_led.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo fonte led.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_controller.h"
#include "hal_types.h"
#include "drv_led.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define LED_GPIO (2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_led";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_led_end];
static drv_led_state_t led_state;

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_led_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_led_set_state(void *parameters);
hal_result_t _drv_led_get_state(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_led_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_led_init;
	_this_drv.function = _this_function;
	_this_function[drv_led_set_state_id] = _drv_led_set_state;
	_this_function[drv_led_get_state_id] = _drv_led_get_state;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_led_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }

    led_state = drv_led_state_off;

	if (gpio_set_level(LED_GPIO, 0) != ESP_OK)
    {
    	return hal_result_fail;
    }

    return hal_result_ok;
}

hal_result_t _drv_led_set_state(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	if (*(drv_led_state_t *)parameters == drv_led_state_on)
	{
		if (gpio_set_level(LED_GPIO, 1) != ESP_OK)
	    {
	    	return hal_result_fail;
	    }
		else
		{
			led_state = drv_led_state_on;
		}
	}
	else
	{
		if (gpio_set_level(LED_GPIO, 0) != ESP_OK)
	    {
	    	return hal_result_fail;
	    }
		else
		{
			led_state = drv_led_state_off;
		}
	}

	return hal_result_ok;
}

hal_result_t _drv_led_get_state(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	*(drv_led_state_t *)parameters = led_state;

	return hal_result_ok;
}


