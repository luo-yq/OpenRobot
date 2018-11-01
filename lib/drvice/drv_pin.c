#include "drv_pin.h"
#include "drv_micros.h"
#ifdef __cplusplus
 extern "C" {
#endif



extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{
	{GPIOA, GPIO_PIN_0, NOT_EXTI},	 //0			 
	{GPIOA, GPIO_PIN_1, NOT_EXTI},	 // 1
	{GPIOA, GPIO_PIN_2, NOT_EXTI},	 // 2
	{GPIOA, GPIO_PIN_3, NOT_EXTI},	 // 3
	{GPIOA, GPIO_PIN_4, NOT_EXTI},	 // 4	 
	{GPIOA, GPIO_PIN_5, NOT_EXTI},	 //5			 
	{GPIOA, GPIO_PIN_6, NOT_EXTI},	 //6			 
	{GPIOA, GPIO_PIN_7, NOT_EXTI},	 //7			 
	{GPIOA, GPIO_PIN_8, NOT_EXTI},	 //8			 
	{GPIOA, GPIO_PIN_9, NOT_EXTI},	 //9			 
	{GPIOA, GPIO_PIN_10,NOT_EXTI},  //10 		 
	{GPIOA, GPIO_PIN_11,NOT_EXTI},  //11 		 
	{GPIOA, GPIO_PIN_12,NOT_EXTI},  //12 		 
	{GPIOA, GPIO_PIN_13,NOT_EXTI},  //13 		 
	{GPIOA, GPIO_PIN_14,NOT_EXTI},  //14 		 
	{GPIOA, GPIO_PIN_15,NOT_EXTI},  //15 		 
	{GPIOB, GPIO_PIN_0, NOT_EXTI},	 //16 		 
	{GPIOB, GPIO_PIN_1, NOT_EXTI},	 //17 		 
	{GPIOB, GPIO_PIN_2, NOT_EXTI},	 //18 		 
	{GPIOB, GPIO_PIN_3, NOT_EXTI},	 //19 		 
	{GPIOB, GPIO_PIN_4, NOT_EXTI},	 //20 		 
	{GPIOB, GPIO_PIN_5, NOT_EXTI},	 //21 		 
	{GPIOB, GPIO_PIN_6, NOT_EXTI},	 //22 		 
	{GPIOB, GPIO_PIN_7, NOT_EXTI},	 //23 		 
	{GPIOB, GPIO_PIN_8, NOT_EXTI},	 //24 		 
	{GPIOB, GPIO_PIN_9, NOT_EXTI},	 //25 		 
	{GPIOB, GPIO_PIN_10,NOT_EXTI},  //26 		 
	{GPIOB, GPIO_PIN_11,NOT_EXTI},  //27 		 
	{GPIOB, GPIO_PIN_12,NOT_EXTI},  //28 		 
	{GPIOB, GPIO_PIN_13,NOT_EXTI},  //29 		 
	{GPIOB, GPIO_PIN_14,NOT_EXTI},  //30 		 
	{GPIOB, GPIO_PIN_15,NOT_EXTI},  //31 		 
	{GPIOC, GPIO_PIN_0, NOT_EXTI},	 //32 		 
	{GPIOC, GPIO_PIN_1, NOT_EXTI},	 //33 		 
	{GPIOC, GPIO_PIN_2, NOT_EXTI},	 //34 		 
	{GPIOC, GPIO_PIN_3, NOT_EXTI},	 //35 		 
	{GPIOC, GPIO_PIN_4, NOT_EXTI},	 //36 		 
	{GPIOC, GPIO_PIN_5, NOT_EXTI},	 //37 		 
	{GPIOC, GPIO_PIN_6, NOT_EXTI},	 //38 		 
	{GPIOC, GPIO_PIN_7, NOT_EXTI},	 //39 		 
	{GPIOC, GPIO_PIN_8, NOT_EXTI},	 //40 		 
	{GPIOC, GPIO_PIN_9, NOT_EXTI},	 //41 		 
	{GPIOC, GPIO_PIN_10,NOT_EXTI},  //42 		 
	{GPIOC, GPIO_PIN_11,NOT_EXTI},  //43 		 
	{GPIOC, GPIO_PIN_12,NOT_EXTI},  //44 		 
	{GPIOC, GPIO_PIN_13,NOT_EXTI},  //45 		 
	{GPIOC, GPIO_PIN_14,NOT_EXTI},  //46 		 
	{GPIOC, GPIO_PIN_15,NOT_EXTI},  //47 		 
	{GPIOD, GPIO_PIN_0, NOT_EXTI},	 //48 		 
	{GPIOD, GPIO_PIN_1, NOT_EXTI},	 //49 		 
	{GPIOD, GPIO_PIN_2, NOT_EXTI},	 //50 		 
	{GPIOD, GPIO_PIN_3, NOT_EXTI},	 //51 		 
	{GPIOD, GPIO_PIN_4, NOT_EXTI},	 //52 		 
	{GPIOD, GPIO_PIN_5, NOT_EXTI},	 //53 		 
	{GPIOD, GPIO_PIN_6, NOT_EXTI},	 //54 		 
	{GPIOD, GPIO_PIN_7, NOT_EXTI},	 //55 		 
	{GPIOD, GPIO_PIN_8, NOT_EXTI},	 //56 		 
	{GPIOD, GPIO_PIN_9, NOT_EXTI},	 //57 		 
	{GPIOD, GPIO_PIN_10,NOT_EXTI},  //58 		 
	{GPIOD, GPIO_PIN_11,NOT_EXTI},  //59 		 
	{GPIOD, GPIO_PIN_12,NOT_EXTI},  //60 Motor1_m1		 
	{GPIOD, GPIO_PIN_13,NOT_EXTI},  //61 Motor1_m2		 
	{GPIOD, GPIO_PIN_14,NOT_EXTI},  //62 Motor2_m1		 
	{GPIOD, GPIO_PIN_15,NOT_EXTI},  //63 Motor2_m2		 
	{GPIOE, GPIO_PIN_0, NOT_EXTI},	 //64 		 
	{GPIOE, GPIO_PIN_1, NOT_EXTI},	 //65 		 
	{GPIOE, GPIO_PIN_2, NOT_EXTI},	 //66 		 
	{GPIOE, GPIO_PIN_3, 3},	 //67 		 
	{GPIOE, GPIO_PIN_4, NOT_EXTI},	 //68 		 
	{GPIOE, GPIO_PIN_5, NOT_EXTI},	 //69 		 
	{GPIOE, GPIO_PIN_6, NOT_EXTI},	 //70 		 
	{GPIOE, GPIO_PIN_7, NOT_EXTI},	 //71 		 
	{GPIOE, GPIO_PIN_8, NOT_EXTI},	 //72 		 
	{GPIOE, GPIO_PIN_9, NOT_EXTI},	 //73 Motor3_m1		 
	{GPIOE, GPIO_PIN_10,NOT_EXTI},  //74 		 
	{GPIOE, GPIO_PIN_11,NOT_EXTI},  //75 Motor3_m2		 
	{GPIOE, GPIO_PIN_12,NOT_EXTI},  //76 		 
	{GPIOE, GPIO_PIN_13,NOT_EXTI},  //77 Motor4_m1		 
	{GPIOE, GPIO_PIN_14,NOT_EXTI},  //78 Motor4_m2			 
	{GPIOE, GPIO_PIN_15,NOT_EXTI},  //79 		 
	{GPIOF, GPIO_PIN_0, 0},	 //80 		 
	{GPIOF, GPIO_PIN_1, 1},	 //81 		 
	{GPIOF, GPIO_PIN_2, 2},	 //82 		 
	{GPIOF, GPIO_PIN_3, NOT_EXTI},	 //83 		 
	{GPIOF, GPIO_PIN_4, NOT_EXTI},	 //84 		 
	{GPIOF, GPIO_PIN_5, NOT_EXTI},	 //85 		 
	{GPIOF, GPIO_PIN_6, NOT_EXTI},	 //86 		 
	{GPIOF, GPIO_PIN_7, NOT_EXTI},	 //87 		 
	{GPIOF, GPIO_PIN_8, 8},	 //88 		 
	{GPIOF, GPIO_PIN_9, NOT_EXTI},	 //89 		 
	{GPIOF, GPIO_PIN_10,10},  //90 		 
	{GPIOF, GPIO_PIN_11,NOT_EXTI},  //91 		 
	{GPIOF, GPIO_PIN_12,NOT_EXTI},  //92 		 
	{GPIOF, GPIO_PIN_13,NOT_EXTI},  //93 		 
	{GPIOF, GPIO_PIN_14,14},  //94 		 
	{GPIOF, GPIO_PIN_15,NOT_EXTI},  //95 		 
	{GPIOG, GPIO_PIN_0, NOT_EXTI},	 //96 		 
	{GPIOG, GPIO_PIN_1, NOT_EXTI},	 //97 		 
	{GPIOG, GPIO_PIN_2, NOT_EXTI},	 //98 		 
	{GPIOG, GPIO_PIN_3, NOT_EXTI},	 //99 		 
	{GPIOG, GPIO_PIN_4, 4},	 //100		 
	{GPIOG, GPIO_PIN_5, 5},	 //101		 
	{GPIOG, GPIO_PIN_6, 6},	 //102		 
	{GPIOG, GPIO_PIN_7, 7},			//103  
	{GPIOG, GPIO_PIN_8, NOT_EXTI},			//104  
	{GPIOG, GPIO_PIN_9, NOT_EXTI},			//105  
	{GPIOG, GPIO_PIN_10,NOT_EXTI}, 		//106  
	{GPIOG, GPIO_PIN_11,NOT_EXTI}, 		//107  
	{GPIOG, GPIO_PIN_12,NOT_EXTI}, 		//108  
	{GPIOG, GPIO_PIN_13,NOT_EXTI}, 		//109  
	{GPIOG, GPIO_PIN_14,NOT_EXTI}, 		//110  
	{GPIOG, GPIO_PIN_15,NOT_EXTI}, 		//111  

																
    {NULL , 0  ,NOT_EXTI}
};

extern void pinMode( uint32_t ulPin, uint32_t ulMode )
{
	GPIO_InitTypeDef  GPIO_InitStruct;


	GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;


	switch ( ulMode )
	{
	  case INPUT:
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    break ;

	  case INPUT_PULLUP:
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			break ;

	  case INPUT_PULLDOWN:
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    break;

	  case OUTPUT:
 			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 			GPIO_InitStruct.Pull = GPIO_NOPULL;
 			break ;

    case OUTPUT_OPEN:
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
 			GPIO_InitStruct.Pull = GPIO_NOPULL;
      break;

	  case INPUT_ANALOG:
	    //drv_adc_pin_init(ulPin);
	    break;

	  default:
	    break ;
	}

	if( ulMode != INPUT_ANALOG )
	{
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct);
	}
}

extern void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{

  switch(ulVal) {

    case HIGH:
			/*
					AVR allows for the writing of inputs to set the pull up state
					we may want to do this here as well to maintain compatibility.
			*/
      HAL_GPIO_WritePin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction,GPIO_PIN_SET);
			break;

    case LOW:
      /* simply reset the pin */
      HAL_GPIO_WritePin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction,GPIO_PIN_RESET);
			break;

    default:
      /* should do an assert here to handle error conditions */
			break;
  }
}

extern int digitalRead( uint32_t ulPin )
{
	if(HAL_GPIO_ReadPin(g_Pin2PortMapArray[ulPin].GPIOx_Port,g_Pin2PortMapArray[ulPin].Pin_abstraction) == GPIO_PIN_RESET)
	{
		return LOW; // Set from HIGH to LOW by Vassilis Serasidis
	}

	return HIGH ; // Set from LOW to HIGH by Vassilis Serasidis
}


uint32_t pulseIn( uint32_t ulPin, uint32_t state, uint32_t timeout )
{
    // cache the port and bit of the pin in order to speed up the
   // pulse width measuring loop and achieve finer resolution.  calling
   // digitalRead() instead yields much coarser resolution.
   uint16_t bit = g_Pin2PortMapArray[ulPin].Pin_abstraction;
   unsigned long stateMask = state ? bit : 0;

   unsigned long startMicros = micros();

   // wait for any previous pulse to end
   while ((g_Pin2PortMapArray[ulPin].GPIOx_Port->IDR & bit) == stateMask)
    if (micros() - startMicros > timeout)
         return 0;

   // wait for the pulse to start
   while ((g_Pin2PortMapArray[ulPin].GPIOx_Port->IDR & bit) != stateMask)
     if (micros() - startMicros > timeout)
         return 0;

   unsigned long start = micros();
   // wait for the pulse to stop
   while ((g_Pin2PortMapArray[ulPin].GPIOx_Port->IDR & bit) == stateMask){
      if (micros() - startMicros > timeout)
         return 0;
   }

   return micros() - start;
}


#ifdef __cplusplus
}
#endif





