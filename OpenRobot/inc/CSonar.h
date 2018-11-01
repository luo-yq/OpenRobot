#ifndef _CSONAR_H
#define _CSONAR_H
#include <string.h>
#include "drv_include.h"
#include "drv_exti.h"
#include "drv_led.h"
#include "UARTClass.h"

typedef struct{
	uint32_t echo;
	uint32_t trig;
	uint64_t distanceData;
	float distance;
}sonar_t;

class CSonar
{

public:
	CSonar();
	~CSonar();
	void Init(UARTClass *serial);
	void update(void);
	void upload(void);
	void clear(void);
private:
	UARTClass *serial;
	bool isRunning;
	char send_buf[256];
	
};

#endif
