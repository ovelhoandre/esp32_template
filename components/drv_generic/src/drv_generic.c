/**
 ******************************************************************************
 * @file    drv_generic.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte de template de driver.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_controller.h"
#include "drv_generic.h"
#include "hal_types.h"
#include "esp_err.h"
#include "esp_log.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_generic";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_generic_end];

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_generic_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_generic_func1(void *parameters);
hal_result_t _drv_generic_func2(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_generic_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_generic_init;
	_this_drv.function = _this_function;
	_this_function[drv_generic_func1_id] = _drv_generic_func1;
	_this_function[drv_generic_func2_id] = _drv_generic_func2;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_generic_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

	return hal_result_ok;
}

hal_result_t _drv_generic_func1(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	return hal_result_ok;
}

hal_result_t _drv_generic_func2(void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	return hal_result_ok;
}

