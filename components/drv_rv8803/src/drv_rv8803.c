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
#include <stdint.h>
#include "drv_controller.h"
#include "hal_types.h"
#include "drv_rv8803.h"
#include "drv_i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define TIME_ARRAY_LENGTH	(8) // Total number of writable values in device
#define TIME_HUNDREDTHS		(0)
#define TIME_SECONDS		(1)
#define TIME_MINUTES		(2)
#define TIME_HOURS			(3)
#define TIME_WEEKDAY		(4)
#define TIME_DATE			(5)
#define TIME_MONTH			(6)
#define TIME_YEAR			(7)
#define RV8803_HUNDREDTHS	(0x10)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "drv_rv8803";
static drv_t _this_drv;
static ptr_func_drv_t _this_function[drv_rv8803_end];
static drv_i2c_rw_conf_t *_rv8803_conf;
static drv_rtc_dst_t _dst;

/* Private functions prototypes ----------------------------------------------*/
hal_result_t _drv_rv8803_init(uint8_t drv_id, void *parameters);
hal_result_t _drv_rv8803_set_time(void *parameters);
hal_result_t _drv_rv8803_get_time(void *parameters);
hal_result_t _drv_rv8803_set_dst(void *parameters);
uint8_t _drv_rv8803_get_wday(struct tm *rtc_time);
uint8_t _BCDtoDEC(uint8_t val);
uint8_t _DECtoBCD(uint8_t val);

/* Public functions ----------------------------------------------------------*/

drv_t *drv_rv8803_get_driver(void)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);

	_this_drv.init = _drv_rv8803_init;
	_this_drv.function = _this_function;
	_this_function[drv_rv8803_set_time_id] = _drv_rv8803_set_time;
	_this_function[drv_rv8803_get_time_id] = _drv_rv8803_get_time;
	_this_function[drv_rv8803_set_dst_id] = _drv_rv8803_set_dst;

	return &_this_drv;
}

/* Private functions ---------------------------------------------------------*/

hal_result_t _drv_rv8803_init(uint8_t drv_id, void *parameters)
{
	ESP_LOGI(TAG, "%s", __FUNCTION__);
	ESP_LOGI(TAG, "_this_drv.id = %d", drv_id);

	_this_drv.id = drv_id;

	_rv8803_conf = parameters;

	if (_rv8803_conf == NULL)
	{
		ESP_LOGE(TAG, "_rv8803_conf == NULL");
		return hal_result_fail;
	}

	_rv8803_conf->data[0] = 0;

	if (drv_call_drv(drv_i2c, drv_i2c_write_byte_id, _rv8803_conf) != hal_result_ok)
	{
		ESP_LOGE(TAG, "drv_i2c_send_byte_id fail");
		return hal_result_fail;
	}

	_dst.dstsday = 0;
	_dst.dsteday = 0;
	_dst.dstsmon = 0;
	_dst.dstemon = 0;

    return hal_result_ok;
}

hal_result_t _drv_rv8803_set_time(void *parameters)
{
	_rv8803_conf->data[TIME_HUNDREDTHS] = 0;
	_rv8803_conf->data[TIME_HOURS] = _DECtoBCD(((drv_rtc_time_t *)parameters)->hour);
	_rv8803_conf->data[TIME_MINUTES] = _DECtoBCD(((drv_rtc_time_t *)parameters)->min);
	_rv8803_conf->data[TIME_SECONDS] = _DECtoBCD(((drv_rtc_time_t *)parameters)->sec);
	_rv8803_conf->data[TIME_WEEKDAY] = _DECtoBCD(((drv_rtc_time_t *)parameters)->wday);
	_rv8803_conf->data[TIME_DATE] = _DECtoBCD(((drv_rtc_time_t *)parameters)->mday);
	_rv8803_conf->data[TIME_MONTH] = _DECtoBCD(((drv_rtc_time_t *)parameters)->mon);
	_rv8803_conf->data[TIME_YEAR] = _DECtoBCD(((drv_rtc_time_t *)parameters)->year);

	_rv8803_conf->command = RV8803_HUNDREDTHS;
	_rv8803_conf->len = TIME_ARRAY_LENGTH;

	if (drv_call_drv(drv_i2c, drv_i2c_write_block_id, _rv8803_conf) != hal_result_ok)
	{
		ESP_LOGE(TAG, "drv_i2c_write_block_id fail");
		return hal_result_fail;
	}

    return hal_result_ok;
}

hal_result_t _drv_rv8803_get_time(void *parameters)
{
	uint16_t start_set, end_set, read;
	struct tm tm, *get_tm;
	time_t tv;

	_rv8803_conf->command = RV8803_HUNDREDTHS;
	_rv8803_conf->len = TIME_ARRAY_LENGTH;

	if (drv_call_drv(drv_i2c, drv_i2c_read_block_id, _rv8803_conf) != hal_result_ok)
	{
		ESP_LOGE(TAG, "drv_i2c_read_block_id fail");
		return hal_result_fail;
	}

	((drv_rtc_time_t *)parameters)->hour = _BCDtoDEC(_rv8803_conf->data[TIME_HOURS]);
	((drv_rtc_time_t *)parameters)->min = _BCDtoDEC(_rv8803_conf->data[TIME_MINUTES]);
	((drv_rtc_time_t *)parameters)->sec = _BCDtoDEC(_rv8803_conf->data[TIME_SECONDS]);
	((drv_rtc_time_t *)parameters)->wday = _BCDtoDEC(_rv8803_conf->data[TIME_WEEKDAY]);
	((drv_rtc_time_t *)parameters)->mday = _BCDtoDEC(_rv8803_conf->data[TIME_DATE]);
	((drv_rtc_time_t *)parameters)->mon = _BCDtoDEC(_rv8803_conf->data[TIME_MONTH]);
	((drv_rtc_time_t *)parameters)->year = _BCDtoDEC(_rv8803_conf->data[TIME_YEAR]);

	/* Verificar se aplica horario de verao */
	start_set = (_dst.dstsmon * 31) + _dst.dstsday;
	end_set = (_dst.dstemon * 31) + _dst.dsteday;
	read = (((drv_rtc_time_t *)parameters)->mon * 31) + ((drv_rtc_time_t *)parameters)->mday;

	if ((read >= start_set) && (read < end_set))
	{
		tm.tm_hour = ((drv_rtc_time_t *)parameters)->hour;
		tm.tm_min = ((drv_rtc_time_t *)parameters)->min;
		tm.tm_sec = ((drv_rtc_time_t *)parameters)->sec;
		tm.tm_year = ((drv_rtc_time_t *)parameters)->year + 100;
		tm.tm_mon = ((drv_rtc_time_t *)parameters)->mon;
		tm.tm_mday = ((drv_rtc_time_t *)parameters)->mday;

		tv = mktime(&tm) + (1 * 3600);
		get_tm = gmtime(&tv);

		((drv_rtc_time_t *)parameters)->hour = get_tm->tm_hour;
		((drv_rtc_time_t *)parameters)->min = get_tm->tm_min;
		((drv_rtc_time_t *)parameters)->sec = get_tm->tm_sec;
		((drv_rtc_time_t *)parameters)->mday = get_tm->tm_mday;
		((drv_rtc_time_t *)parameters)->mon = get_tm->tm_mon;
		get_tm->tm_year -= 100;
		((drv_rtc_time_t *)parameters)->year = get_tm->tm_year;
		((drv_rtc_time_t *)parameters)->wday = _drv_rv8803_get_wday(get_tm);
	}

    return hal_result_ok;
}

hal_result_t _drv_rv8803_set_dst(void *parameters)
{
	_dst.dstsday = ((drv_rtc_dst_t *)parameters)->dstsday;
	_dst.dsteday = ((drv_rtc_dst_t *)parameters)->dsteday;
	_dst.dstsmon = ((drv_rtc_dst_t *)parameters)->dstsmon;
	_dst.dstemon = ((drv_rtc_dst_t *)parameters)->dstemon;

	return hal_result_ok;
}

uint8_t _drv_rv8803_get_wday(struct tm *rtc_time)
{
	int d = rtc_time->tm_mday;
	int m = rtc_time->tm_mon;
	int y = rtc_time->tm_year + 2000;
    static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return (1 << (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7);
}

uint8_t _BCDtoDEC(uint8_t val)
{
    return ((val / 0x10) * 10) + (val % 0x10);
}

uint8_t _DECtoBCD(uint8_t val)
{
    return ((val / 10) * 0x10) + (val % 10);
}
