/*
 *  drv_battry.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "drv_battery.h"
//-- internal definition
//

ADC_HandleTypeDef hADC3;
uint32_t BatteryPin = 32;//PC0 => 32

int drv_adc_init()
{
  hADC3.Instance                   = ADC3;
  hADC3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hADC3.Init.Resolution            = ADC_RESOLUTION_12B;
  hADC3.Init.ScanConvMode          = DISABLE;
  hADC3.Init.ContinuousConvMode    = DISABLE;
  hADC3.Init.DiscontinuousConvMode = DISABLE;
  hADC3.Init.NbrOfDiscConversion   = 0;
  hADC3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hADC3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hADC3.Init.NbrOfConversion       = 1;
  hADC3.Init.DMAContinuousRequests = DISABLE;
  hADC3.Init.EOCSelection          = DISABLE;

  if (HAL_ADC_Init(&hADC3) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void drv_battery_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  drv_adc_init();


  HAL_GPIO_DeInit(g_Pin2PortMapArray[BatteryPin].GPIOx_Port, g_Pin2PortMapArray[BatteryPin].Pin_abstraction);

  GPIO_InitStruct.Pin = g_Pin2PortMapArray[BatteryPin].Pin_abstraction;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(g_Pin2PortMapArray[BatteryPin].GPIOx_Port, &GPIO_InitStruct);
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

uint32_t getBatteryVol()
{
	ADC_ChannelConfTypeDef sConfig;
  ADC_HandleTypeDef      *hADCx;
	uint32_t ulValue = 0;
  uint32_t ulChannel;

  ulChannel = ADC_CHANNEL_10;
  

  hADCx = &hADC3;

  sConfig.Channel      = ulChannel;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset       = 0;
  HAL_ADC_ConfigChannel(hADCx, &sConfig);
  
  HAL_ADC_Start(hADCx);
  HAL_ADC_PollForConversion(hADCx, 10);
  ulValue = HAL_ADC_GetValue(hADCx)*330/4096;

  //ulValue = mapResolution(ulValue, 12, 10);
 
  return ulValue;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;

	if( hadc->Instance == ADC3 )
	{
		__HAL_RCC_ADC3_CLK_ENABLE();
	}

	HAL_GPIO_DeInit(g_Pin2PortMapArray[BatteryPin].GPIOx_Port, g_Pin2PortMapArray[BatteryPin].Pin_abstraction);

	GPIO_InitStruct.Pin = g_Pin2PortMapArray[BatteryPin].Pin_abstraction;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(g_Pin2PortMapArray[BatteryPin].GPIOx_Port, &GPIO_InitStruct);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{



    if( hadc->Instance == ADC3 )
    {
      __HAL_RCC_ADC3_CLK_DISABLE();
    }
    if( hadc->Instance == ADC1 )
    {
      __HAL_RCC_ADC1_CLK_DISABLE();
    }

    HAL_GPIO_DeInit(g_Pin2PortMapArray[BatteryPin].GPIOx_Port, g_Pin2PortMapArray[BatteryPin].Pin_abstraction);
}



