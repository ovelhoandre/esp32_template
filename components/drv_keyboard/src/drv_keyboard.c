/**
 ******************************************************************************
 * @file    drv_keyboard.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte do teclado matricial.
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
#include "drv_keyboard.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

// Inputs
#define KB_COL_0	(35)
#define KB_COL_1	(34)
#define KB_COL_2	(39)
#define KB_COL_3	(36)

// Outputs
#define KB_ROW_0	(18)
#define KB_ROW_1	(17)
#define KB_ROW_2	(16)
#define KB_ROW_3	(4)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_keyboard";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_keyboard_end];
static drv_keyboard_t _keyboard;

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_keyboard_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_keyboard_get(void *parameters);
static void _drv_keyboard_task(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_keyboard_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_keyboard_init;
	_this_drv.function = _this_function;
	_this_function[drv_keyboard_get_id] = _drv_keyboard_get;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_keyboard_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

	_keyboard.raw = 0;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << KB_COL_0);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_COL_1);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_COL_2);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_COL_3);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << KB_ROW_0);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_ROW_1);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_ROW_2);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }
    io_conf.pin_bit_mask = (1ULL << KB_ROW_3);
    if (gpio_config(&io_conf) != ESP_OK)
    {
    	return hal_result_fail;
    }

	if (xTaskCreate(_drv_keyboard_task, "_drv_keyboard_task", 2048, NULL, 3, NULL) != pdTRUE)
	{
		return hal_result_fail;
	}

	return hal_result_ok;
}

hal_result_t _drv_keyboard_get(void *parameters)
{
	((drv_keyboard_t *)parameters)->raw = _keyboard.raw;

	return hal_result_ok;
}

static void _drv_keyboard_task(void *parameters)
{
	esp_task_wdt_add(NULL);

	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, 50 / portTICK_PERIOD_MS);

		// Varredura linha 0
		gpio_set_level(KB_ROW_0, 0);
		gpio_set_level(KB_ROW_1, 1);
		gpio_set_level(KB_ROW_2, 1);
		gpio_set_level(KB_ROW_3, 1);

		if (gpio_get_level(KB_COL_0) == 0)
		{
			_keyboard.key.k1 = 1;
		}
		else
		{
			_keyboard.key.k1 = 0;
		}

		if (gpio_get_level(KB_COL_1) == 0)
		{
			_keyboard.key.k2 = 1;
		}
		else
		{
			_keyboard.key.k2 = 0;
		}

		if (gpio_get_level(KB_COL_2) == 0)
		{
			_keyboard.key.k3 = 1;
		}
		else
		{
			_keyboard.key.k3 = 0;
		}

		if (gpio_get_level(KB_COL_3) == 0)
		{
			_keyboard.key.kmenu = 1;
		}
		else
		{
			_keyboard.key.kmenu = 0;
		}

		// Varredura linha 1
		gpio_set_level(KB_ROW_0, 1);
		gpio_set_level(KB_ROW_1, 0);
		gpio_set_level(KB_ROW_2, 1);
		gpio_set_level(KB_ROW_3, 1);

		if (gpio_get_level(KB_COL_0) == 0)
		{
			_keyboard.key.k4 = 1;
		}
		else
		{
			_keyboard.key.k4 = 0;
		}

		if (gpio_get_level(KB_COL_1) == 0)
		{
			_keyboard.key.k5 = 1;
		}
		else
		{
			_keyboard.key.k5 = 0;
		}

		if (gpio_get_level(KB_COL_2) == 0)
		{
			_keyboard.key.k6 = 1;
		}
		else
		{
			_keyboard.key.k6 = 0;
		}

		if (gpio_get_level(KB_COL_3) == 0)
		{
			_keyboard.key.kdel = 1;
		}
		else
		{
			_keyboard.key.kdel = 0;
		}

		// Varredura linha 2
		gpio_set_level(KB_ROW_0, 1);
		gpio_set_level(KB_ROW_1, 1);
		gpio_set_level(KB_ROW_2, 0);
		gpio_set_level(KB_ROW_3, 1);

		if (gpio_get_level(KB_COL_0) == 0)
		{
			_keyboard.key.k7 = 1;
		}
		else
		{
			_keyboard.key.k7 = 0;
		}

		if (gpio_get_level(KB_COL_1) == 0)
		{
			_keyboard.key.k8 = 1;
		}
		else
		{
			_keyboard.key.k8 = 0;
		}

		if (gpio_get_level(KB_COL_2) == 0)
		{
			_keyboard.key.k9 = 1;
		}
		else
		{
			_keyboard.key.k9 = 0;
		}

		if (gpio_get_level(KB_COL_3) == 0)
		{
			_keyboard.key.kup = 1;
		}
		else
		{
			_keyboard.key.kup = 0;
		}

		// Varredura linha 3
		gpio_set_level(KB_ROW_0, 1);
		gpio_set_level(KB_ROW_1, 1);
		gpio_set_level(KB_ROW_2, 1);
		gpio_set_level(KB_ROW_3, 0);

		if (gpio_get_level(KB_COL_0) == 0)
		{
			_keyboard.key.kesc = 1;
		}
		else
		{
			_keyboard.key.kesc = 0;
		}

		if (gpio_get_level(KB_COL_1) == 0)
		{
			_keyboard.key.k0 = 1;
		}
		else
		{
			_keyboard.key.k0 = 0;
		}

		if (gpio_get_level(KB_COL_2) == 0)
		{
			_keyboard.key.kok = 1;
		}
		else
		{
			_keyboard.key.kok = 0;
		}

		if (gpio_get_level(KB_COL_3) == 0)
		{
			_keyboard.key.kdown = 1;
		}
		else
		{
			_keyboard.key.kdown = 0;
		}

		if (_keyboard.key.k0)
		{
			_keyboard.key.value = 0;
		}
		else if (_keyboard.key.k1)
		{
			_keyboard.key.value = 1;
		}
		else if (_keyboard.key.k2)
		{
			_keyboard.key.value = 2;
		}
		else if (_keyboard.key.k3)
		{
			_keyboard.key.value = 3;
		}
		else if (_keyboard.key.k4)
		{
			_keyboard.key.value = 4;
		}
		else if (_keyboard.key.k5)
		{
			_keyboard.key.value = 5;
		}
		else if (_keyboard.key.k6)
		{
			_keyboard.key.value = 6;
		}
		else if (_keyboard.key.k7)
		{
			_keyboard.key.value = 7;
		}
		else if (_keyboard.key.k8)
		{
			_keyboard.key.value = 8;
		}
		else if (_keyboard.key.k9)
		{
			_keyboard.key.value = 9;
		}
		else
		{
			_keyboard.key.value = 0;
		}

		if (_keyboard.key.k0) ESP_LOGI(TAG, "keyboard.key.k0 = %d", _keyboard.key.k0);
		if (_keyboard.key.k1) ESP_LOGI(TAG, "keyboard.key.k1 = %d", _keyboard.key.k1);
		if (_keyboard.key.k2) ESP_LOGI(TAG, "keyboard.key.k2 = %d", _keyboard.key.k2);
		if (_keyboard.key.k3) ESP_LOGI(TAG, "keyboard.key.k3 = %d", _keyboard.key.k3);
		if (_keyboard.key.k4) ESP_LOGI(TAG, "keyboard.key.k4 = %d", _keyboard.key.k4);
		if (_keyboard.key.k5) ESP_LOGI(TAG, "keyboard.key.k5 = %d", _keyboard.key.k5);
		if (_keyboard.key.k6) ESP_LOGI(TAG, "keyboard.key.k6 = %d", _keyboard.key.k6);
		if (_keyboard.key.k7) ESP_LOGI(TAG, "keyboard.key.k7 = %d", _keyboard.key.k7);
		if (_keyboard.key.k8) ESP_LOGI(TAG, "keyboard.key.k8 = %d", _keyboard.key.k8);
		if (_keyboard.key.k9) ESP_LOGI(TAG, "keyboard.key.k9 = %d", _keyboard.key.k9);
		if (_keyboard.key.value) ESP_LOGI(TAG, "keyboard.key.value = %d", _keyboard.key.value);
		if (_keyboard.key.kmenu) ESP_LOGI(TAG, "keyboard.key.kmenu = %d", _keyboard.key.kmenu);
		if (_keyboard.key.kdel) ESP_LOGI(TAG, "keyboard.key.kdel = %d", _keyboard.key.kdel);
		if (_keyboard.key.kesc) ESP_LOGI(TAG, "keyboard.key.kesc = %d", _keyboard.key.kesc);
		if (_keyboard.key.kok) ESP_LOGI(TAG, "keyboard.key.kok = %d", _keyboard.key.kok);
		if (_keyboard.key.kup) ESP_LOGI(TAG, "keyboard.key.kup = %d", _keyboard.key.kup);
		if (_keyboard.key.kdown) ESP_LOGI(TAG, "keyboard.key.kdown = %d", _keyboard.key.kdown);

		esp_task_wdt_reset();
	}
}

