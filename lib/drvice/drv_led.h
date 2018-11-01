#ifndef _DRV_LED_H
#define _DRV_LED_H

#ifdef __cplusplus
	extern "C" {
#endif
//#include <intrinsics.h>
#include "sys.h"

#define LED_ON(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_RESET))
#define LED_OFF(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_SET))

#define BEEP_ONCE		{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);delay(100);HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);}
#define BEEP_LONG		{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);delay(500);HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);}
#define BEEP_ON			{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);}
#define BEEP_OFF		{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);}

#define LED_TOGGLE(n) ((n>5) ? __nop() : HAL_GPIO_TogglePin(GPIOC, (uint16_t)(GPIO_PIN_1<<n))) 


void drv_Led_Init(void);

#ifdef __cplusplus
 }
#endif

#endif
 
