/*
 *  drv_uart.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H


#ifdef __cplusplus
 extern "C" {
#endif

#include "drv_include.h"

#define MOTOR_CH1 0
#define MOTOR_CH2 1
#define MOTOR_CH3 2
#define MOTOR_CH4 3
#define MOTOR_CH5 4
#define MOTOR_CH6 5
#define MOTOR_CH7 6
#define MOTOR_CH8 7





void drv_motor_pwm_init();
void drv_pwm_set_duty(uint32_t ulPin, uint32_t ulDuty );


#ifdef __cplusplus
}
#endif


#endif
