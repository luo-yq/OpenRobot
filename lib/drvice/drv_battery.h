/*
 *  drv_uart.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_BATTRY_H
#define DRV_BATTRY_H


#ifdef __cplusplus
 extern "C" {
#endif


//#include "def.h"
#include "drv_include.h"
#include "stm32f7xx.h"
#include "core_cm7.h"
#include "stm32f7xx_hal.h"

void drv_battery_init(void);
uint32_t getBatteryVol();

#ifdef __cplusplus
}
#endif


#endif
