/**
 ******************************************************************************
 * @file    drv_relay.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo fonte relay.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_controller.h"
#include "drv_relay.h"
#include "hal_global.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define RELAY_0_GPIO	(32)
#define RELAY_1_GPIO	(33)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_relay";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_relay_end];
static drv_relay_state_t _relay_num_0_state, _relay_num_1_state;

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_relay_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_relay_set_state(void *parameters);
hal_result_t _drv_relay_get_state(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_relay_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_relay_init;
	_this_drv.function = _this_function;
	_this_function[drv_relay_set_state_id] = _drv_relay_set_state;
	_this_function[drv_relay_get_state_id] = _drv_relay_get_state;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_relay_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_0_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }

    io_conf.pin_bit_mask = (1ULL << RELAY_1_GPIO);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }

    _relay_num_0_state = drv_relay_state_off;
    _relay_num_1_state = drv_relay_state_off;

	if (gpio_set_level(RELAY_0_GPIO, 0) != ESP_OK)
	{
		return hal_result_fail;
	}

	if (gpio_set_level(RELAY_1_GPIO, 0) != ESP_OK)
	{
		return hal_result_fail;
	}

    return hal_result_ok;
}

hal_result_t _drv_relay_set_state(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	if (((drv_relay_conf_t *)parameters)->relay_num == drv_relay_num_0)
	{
		if (((drv_relay_conf_t *)parameters)->relay_state == drv_relay_state_on)
		{
			if (gpio_set_level(RELAY_0_GPIO, 1) != ESP_OK)
			{
				return hal_result_fail;
			}
			else
			{
				_relay_num_0_state = drv_relay_state_on;
			}
		}
		else
		{
			if (gpio_set_level(RELAY_0_GPIO, 0) != ESP_OK)
			{
				return hal_result_fail;
			}
			else
			{
				_relay_num_0_state = drv_relay_state_off;
			}
		}
	}
	else if (((drv_relay_conf_t *)parameters)->relay_num == drv_relay_num_1)
	{
		if (((drv_relay_conf_t *)parameters)->relay_state == drv_relay_state_on)
		{
			if (gpio_set_level(RELAY_1_GPIO, 1) != ESP_OK)
			{
				return hal_result_fail;
			}
			else
			{
				_relay_num_1_state = drv_relay_state_on;
			}
		}
		else
		{
			if (gpio_set_level(RELAY_1_GPIO, 0) != ESP_OK)
			{
				return hal_result_fail;
			}
			else
			{
				_relay_num_1_state = drv_relay_state_off;
			}
		}
	}
	else
	{
		return hal_result_fail;
	}

	return hal_result_ok;
}

hal_result_t _drv_relay_get_state(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	if (((drv_relay_conf_t *)parameters)->relay_num == drv_relay_num_0)
	{
		((drv_relay_conf_t *)parameters)->relay_state = _relay_num_0_state;
	}
	else if (((drv_relay_conf_t *)parameters)->relay_num == drv_relay_num_1)
	{
		((drv_relay_conf_t *)parameters)->relay_state = _relay_num_1_state;
	}
	else
	{
		return hal_result_fail;
	}

	return hal_result_ok;
}



