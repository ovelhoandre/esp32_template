/**
 ******************************************************************************
 * @file    drv_i2c.c
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Segunda, 10 de abril de 2023
 * @brief   Arquivo fonte de driver i2c.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "drv_controller.h"
#include "hal_types.h"
#include "drv_i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define I2C_MASTER_TX_BUF_DISABLE   (0) /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   (0) /*!< I2C master doesn't need buffer */

#define WRITE_BIT      I2C_MASTER_WRITE
#define READ_BIT       I2C_MASTER_READ
#define ACK_CHECK      true
#define NO_ACK_CHECK   false
#define ACK_VALUE      0x0
#define NACK_VALUE     0x1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_i2c";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_i2c_end];

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_i2c_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_i2c_write_byte(void *parameters);
hal_result_t _drv_i2c_read_byte(void *parameters);
hal_result_t _drv_i2c_write_cmd(void *parameters);
hal_result_t _drv_i2c_read_cmd(void *parameters);
hal_result_t _drv_i2c_write_block(void *parameters);
hal_result_t _drv_i2c_read_block(void *parameters);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_i2c_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_i2c_init;
	_this_drv.function = _this_function;
	_this_function[drv_i2c_write_byte_id] = _drv_i2c_write_byte;
	_this_function[drv_i2c_read_byte_id] = _drv_i2c_read_byte;
	_this_function[drv_i2c_write_cmd_id] = _drv_i2c_write_cmd;
	_this_function[drv_i2c_read_cmd_id] = _drv_i2c_read_cmd;
	_this_function[drv_i2c_write_block_id] = _drv_i2c_write_block;
	_this_function[drv_i2c_read_block_id] = _drv_i2c_read_block;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_i2c_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ((drv_i2c_init_conf_t *)parameters)->sda_io_pin,
        .scl_io_num = ((drv_i2c_init_conf_t *)parameters)->scl_io_pin,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = ((drv_i2c_init_conf_t *)parameters)->clock,
    };

    if (i2c_param_config(((drv_i2c_init_conf_t *)parameters)->i2c_num, &conf) != ESP_OK)
    {
    	ESP_LOGE(TAG, "i2c_param_config fail");
    	return hal_result_fail;
    }

    if (i2c_driver_install(((drv_i2c_init_conf_t *)parameters)->i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0) != ESP_OK)
    {
    	ESP_LOGE(TAG, "i2c_driver_install fail");
    	return hal_result_fail;
    }

	return hal_result_ok;
}

hal_result_t _drv_i2c_write_byte(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->data[0], ACK_CHECK);
    i2c_master_stop(cmd);
	if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
	{

		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
	}
	i2c_cmd_link_delete(cmd);

	return hal_result;
}

hal_result_t _drv_i2c_read_byte(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | READ_BIT, ACK_CHECK);
    i2c_master_read_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->data, NACK_VALUE);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
    {
		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
    }
    i2c_cmd_link_delete(cmd);

	return hal_result;
}

hal_result_t _drv_i2c_write_cmd(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->command, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->data[0], ACK_CHECK);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
    {
		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
    }
    i2c_cmd_link_delete(cmd);

	return hal_result;
}

hal_result_t _drv_i2c_read_cmd(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->command, ACK_CHECK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | READ_BIT, ACK_CHECK);
    i2c_master_read_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->data, NACK_VALUE);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
    {
		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
    }
    i2c_cmd_link_delete(cmd);

	return hal_result;
}

hal_result_t _drv_i2c_write_block(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->command, ACK_CHECK);
    i2c_master_write(cmd, ((drv_i2c_rw_conf_t *)parameters)->data, ((drv_i2c_rw_conf_t *)parameters)->len, ACK_CHECK);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
    {
		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
    }
    i2c_cmd_link_delete(cmd);

	return hal_result;
}

hal_result_t _drv_i2c_read_block(void *parameters)
{
	hal_result_t hal_result = hal_result_ok;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->command, ACK_CHECK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((drv_i2c_rw_conf_t *)parameters)->slave_addr << 1 | READ_BIT, ACK_CHECK);
    if (((drv_i2c_rw_conf_t *)parameters)->len > 1)
    {
        i2c_master_read(cmd, ((drv_i2c_rw_conf_t *)parameters)->data, ((drv_i2c_rw_conf_t *)parameters)->len - 1, ACK_VALUE);
    }
    i2c_master_read_byte(cmd, &((drv_i2c_rw_conf_t *)parameters)->data[((drv_i2c_rw_conf_t *)parameters)->len - 1], NACK_VALUE);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(((drv_i2c_rw_conf_t *)parameters)->i2c_num, cmd, ((drv_i2c_rw_conf_t *)parameters)->timeout) != ESP_OK)
    {
		ESP_LOGE(TAG, "%s fail", __FUNCTION__);
		hal_result = hal_result_fail;
    }
    i2c_cmd_link_delete(cmd);

	return hal_result;
}

