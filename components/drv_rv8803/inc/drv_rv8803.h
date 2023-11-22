/**
 ******************************************************************************
 * @file    drv_rv8803.h
 * @author  Eng. Eletricista Andre L. A. Lopes
 * @version V1.0.0
 * @date    Quarta, 11 de maio de 2022
 * @brief   Arquivo header do RTC.
 ******************************************************************************
 * @attention
 *
 * 					e-mail: andrelopes.al@gmail.com.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DRV_RV8803_H
#define _DRV_RV8803_H

/* Includes ------------------------------------------------------------------*/
#include "hal_types.h"
#include "drv_controller.h"

/* Exported constants --------------------------------------------------------*/
#define WDAY_SUNDAY_FLAG	(1 << 0)
#define WDAY_MONDAY_FLAG	(1 << 1)
#define WDAY_TUESDAY_FLAG	(1 << 2)
#define WDAY_WEDNESDAY_FLAG	(1 << 3)
#define WDAY_THURSDAY_FLAG	(1 << 4)
#define WDAY_FRIDAY_FLAG	(1 << 5)
#define WDAY_SATURDAY_FLAG	(1 << 6)

/* Exported types ------------------------------------------------------------*/
enum
{
	drv_rv8803_set_time_id,
	drv_rv8803_get_time_id,
	drv_rv8803_set_dst_id,
	drv_rv8803_end
};

typedef struct
{
  uint8_t	sec;
  uint8_t	min;
  uint8_t	hour;
  uint8_t	mday;
  uint8_t	mon;
  uint8_t	year;
  uint8_t	wday;
  uint8_t	yday;
} drv_rtc_time_t;

typedef struct
{
	uint8_t	dstsday;	// Dia de inicio horario de verao
	uint8_t	dsteday;	// Dia de fim do horario de verao
	uint8_t	dstsmon;	// Mes de inicio horario de verao
	uint8_t	dstemon;	// Mes de fim do horario de verao
} drv_rtc_dst_t;

/* Exported macro ------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
drv_t *drv_rv8803_get_driver(void);

#endif /* _DRV_RV8803_H */
