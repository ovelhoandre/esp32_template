/**
 ******************************************************************************
 * @file    hal_pcf8574.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo fonte IO Expander.
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
#include "drv_pcf8574.h"
#include "drv_i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_pcf8574";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_pcf8574_end];
static drv_i2c_rw_conf_t *_pcf8574_conf;

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_pcf8574_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_pcf8574_write(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_pcf8574_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_pcf8574_init;
	_this_drv.function = _this_function;
	_this_function[drv_pcf8574_write_id] = _drv_pcf8574_write;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_pcf8574_init(uint8_t drv_id, void *parameters)
{
	drv_pcf8574_port_t port;

	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

	_pcf8574_conf = parameters;

	if (_pcf8574_conf == NULL)
	{
		ESP_LOGE(TAG, "_pcf8574_conf == NULL");
		return hal_result_fail;
	}

	port.raw = 0;
	if (_drv_pcf8574_write(&port) != hal_result_ok)
	{
		ESP_LOGE(TAG, "drv_pcf8574_write fail");
		return hal_result_fail;
	}

	ESP_LOGI(TAG, "%s ok", __FUNCTION__);

    return hal_result_ok;
}

hal_result_t _drv_pcf8574_write(void *parameters)
{
	_pcf8574_conf->data[0] = ((drv_pcf8574_port_t *)parameters)->raw;

	return drv_call_drv(drv_i2c, drv_i2c_write_byte_id, _pcf8574_conf);
}


