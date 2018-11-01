/*
 *  drv_exti.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_EXTI_H
#define DRV_EXTI_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "drv_include.h"
#include "bsp.h"




int drv_exti_init();

//void drv_exti_attach( uint32_t ulPin, void (*callback)(void), uint32_t mode );
//void drv_exti_detach( uint32_t ulPin );
void attachInterrupt( uint32_t pin, voidFuncPtr callback, uint32_t ulMode );

#ifdef __cplusplus
}
#endif


#endif
