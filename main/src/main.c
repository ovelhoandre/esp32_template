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
#include "hal_types.h"
#include "hal_global.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "drv_controller.h"
#include "drv_led.h"
#include "drv_relay.h"
#include "drv_i2c.h"
#include "drv_keyboard.h"
#include "drv_pcf8574.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "main";
static drv_led_state_t _led_state;
static hal_global_data_t *global_data;
static drv_relay_conf_t relay_conf;

/* Private functions prototypes ----------------------------------------------*/
void test_led(void);
void test_relay(void);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Main
 * @param  None
 * @retval None
 */
void app_main(void)
{
	ESP_LOGI(TAG, "Global project init");
	DRV_ERROR_CHECK(hal_global_init());
	global_data = hal_global_get_data();

    ESP_LOGI(TAG, "Driver controller init");
    DRV_ERROR_CHECK(drv_init_controller());

	ESP_LOGI(TAG, "LED driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_led, NULL));

	ESP_LOGI(TAG, "Relay driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_relay, NULL));

	ESP_LOGI(TAG, "i2c driver init");
	global_data->i2c_init_conf.clock = 100000;
	global_data->i2c_init_conf.i2c_num = 0;
	global_data->i2c_init_conf.scl_io_pin = 22;
	global_data->i2c_init_conf.sda_io_pin = 21;
	DRV_ERROR_CHECK(drv_init_drv(drv_i2c, &global_data->i2c_init_conf));

	ESP_LOGI(TAG, "PCF8574 driver init");
	global_data->pcf8574_rw_conf.i2c_num = 0;
	global_data->pcf8574_rw_conf.slave_addr = 0x27;
	global_data->pcf8574_rw_conf.timeout = 1000;
	DRV_ERROR_CHECK(drv_init_drv(drv_pcf8574, &global_data->pcf8574_rw_conf));

	ESP_LOGI(TAG, "Keyboard driver init");
	DRV_ERROR_CHECK(drv_init_drv(drv_keyboard, NULL));

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

	while (1)
	{
		/* Teste driver led */
		test_led();

		/* Teste driver relay */
		//test_relay();

		vTaskDelay(1000 / portTICK_PERIOD_MS);

		esp_task_wdt_reset();
	}
}

void test_led(void)
{
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
}

void test_relay(void)
{
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
}



