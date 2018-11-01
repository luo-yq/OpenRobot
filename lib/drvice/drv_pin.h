#ifndef _DRV_PIN_H
#define _DRV_PIN_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"
#include "stdio.h"	
#include "stdbool.h"

#define NOT_EXTI 0xFF
	 
#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_ANALOG    0x3
#define INPUT_PULLDOWN  0x4
#define OUTPUT_OPEN     0x5

//#define true 0x1
//#define false 0x0

#define PI              3.1415926535897932384626433832795
#define HALF_PI         1.5707963267948966192313216916398
#define TWO_PI          6.283185307179586476925286766559
#define DEG_TO_RAD      0.017453292519943295769236907684886
#define RAD_TO_DEG      57.295779513082320876798154814105
#define EULER           2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

enum BitOrder {
	LSBFIRST = 0,
	MSBFIRST = 1
};

//      LOW 0
//      HIGH 1
#define CHANGE 2
#define FALLING 3
#define RISING 4

#define DEFAULT 1
#define EXTERNAL 0

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

#define abs(x)                  ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
#define sq(x)                   ((x)*(x))



#define interrupts()            __enable_irq()
#define noInterrupts()          __disable_irq()

#define lowByte(w)              ((uint8_t) ((w) & 0xff))
#define highByte(w)             ((uint8_t) ((w) >> 8))

#define bitRead(value, bit)     (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)      ((value) |= (1UL << (bit)))
#define bitClear(value, bit)    ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef bool boolean ;
typedef bool BOOL;
#define FALSE false
#define TRUE true
typedef uint8_t byte ;


typedef struct _Pin2PortMapArray
{
  	GPIO_TypeDef *GPIOx_Port;

  	uint32_t 	Pin_abstraction;
	
	uint32_t 	extiChannel;
} Pin2PortMapArray ;


extern const Pin2PortMapArray g_Pin2PortMapArray[] ;

extern void pinMode( uint32_t ulPin, uint32_t ulMode );
extern void digitalWrite( uint32_t ulPin, uint32_t ulVal );
extern int digitalRead( uint32_t ulPin );
uint32_t pulseIn( uint32_t ulPin, uint32_t state, uint32_t timeout );

#ifdef __cplusplus
}
#endif

#endif
