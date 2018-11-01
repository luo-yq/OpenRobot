/*
 *  drv_uart.h
 *
 *  Created on: 2016. 7.13.
 *      Author: Baram, PBHP
 */

#ifndef DRV_TIMER_H
#define DRV_TIMER_H


#ifdef __cplusplus
 extern "C" {
#endif


#define TIMER_CH_MAX  5

#define TIMER_CH1     0
#define TIMER_CH2     1
#define TIMER_CH3     2
#define TIMER_CH4     3
#define TIMER_TONE    3
#define TIMER_USB     4

typedef void (*voidFuncPtr)(void);


int drv_timer_init();

void drv_timer_pause(uint8_t channel);
void drv_timer_set_period(uint8_t channel, uint32_t period_data);
void drv_timer_attachInterrupt(uint8_t channel, voidFuncPtr handler);
void drv_timer_detachInterrupt(uint8_t channel);
void drv_timer_refresh(uint8_t channel);
void drv_timer_resume(uint8_t channel);


#ifdef __cplusplus
}
#endif


#endif
