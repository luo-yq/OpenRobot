#include "CSonar.h"

#define SONAR_START 0
#define SONAR_END 1

#define SONAR1_ECHO  67//PE3 4*16+3
#define SONAR1_TRIG  66//PE2 4*16+2

#define SONAR2_ECHO  94//PF14 5*16+14
#define SONAR2_TRIG  93//PF13 5*16+13

#define SONAR3_ECHO  90//PF10 5*16+10
#define SONAR3_TRIG  91//PF11 5*16+11

#define SONAR4_ECHO  88//PF8 5*16+8
#define SONAR4_TRIG  89//PF9 5*16+9

#define SONAR5_ECHO  100//PG4 6*16+4
#define SONAR5_TRIG  56//PD8 3*16+8

#define SONAR6_ECHO  101//PG5 4*16+3
#define SONAR6_TRIG  57//PD9 4*16+2

#define SONAR7_ECHO  102//PG6 4*16+3
#define SONAR7_TRIG  58//PD10 4*16+2

#define SONAR8_ECHO  103//PG7 4*16+3
#define SONAR8_TRIG  59//PD11 4*16+2

static sonar_t sonar[8];

extern "C"{
void sonar_handle(sonar_t *sonar)
{
	if(digitalRead(sonar->echo) == GPIO_PIN_SET){
		sonar->distanceData = micros();
	}else{
		if( sonar->distanceData!=0 ){
			sonar->distanceData = micros() - sonar->distanceData;
		}
	}
}

void sonar1_handle(void)
{
	sonar_handle(&sonar[0]);
	LED_TOGGLE(1);
}

void sonar2_handle(void)
{
	sonar_handle(&sonar[1]);
}

void sonar3_handle(void)
{
	sonar_handle(&sonar[2]);
}

void sonar4_handle(void)
{
	sonar_handle(&sonar[3]);
}

void sonar5_handle(void)
{
	sonar_handle(&sonar[4]);
}

void sonar6_handle(void)
{
	sonar_handle(&sonar[5]);
}

void sonar7_handle(void)
{
	sonar_handle(&sonar[6]);
}

void sonar8_handle(void)
{
	sonar_handle(&sonar[7]);
}

}


CSonar::CSonar()
{
	isRunning = false;
	
	sonar[0].echo = SONAR1_ECHO;
	sonar[0].trig = SONAR1_TRIG;
	
	sonar[1].echo = SONAR2_ECHO;
	sonar[1].trig = SONAR2_TRIG;
	
	sonar[2].echo = SONAR3_ECHO;
	sonar[2].trig = SONAR3_TRIG;
	
	sonar[3].echo = SONAR4_ECHO;
	sonar[3].trig = SONAR4_TRIG;
	
	sonar[4].echo = SONAR5_ECHO;
	sonar[4].trig = SONAR5_TRIG;
	
	sonar[5].echo = SONAR6_ECHO;
	sonar[5].trig = SONAR6_TRIG;
	
	sonar[6].echo = SONAR7_ECHO;
	sonar[6].trig = SONAR7_TRIG;
	
	sonar[7].echo = SONAR8_ECHO;
	sonar[7].trig = SONAR8_TRIG;
	
	
	
	sonar[0].distanceData = 0;
	sonar[0].distance = 0.0f;
	
	sonar[1].distanceData = 0;
	sonar[1].distance = 0.0f;
	
	sonar[2].distanceData = 0;
	sonar[2].distance = 0.0f;
	
	sonar[3].distanceData = 0;
	sonar[3].distance = 0.0f;
	
	sonar[4].distanceData = 0;
	sonar[4].distance = 0.0f;
	
	sonar[5].distanceData = 0;
	sonar[5].distance = 0.0f;
	
	sonar[6].distanceData = 0;
	sonar[6].distance = 0.0f;
	
	sonar[7].distanceData = 0;
	sonar[7].distance = 0.0f;
	
}

CSonar::~CSonar()
{
	
}

void CSonar::Init(UARTClass *serial)
{
	pinMode(SONAR1_ECHO,INPUT);
	pinMode(SONAR1_TRIG,OUTPUT);
	attachInterrupt(3, sonar1_handle, CHANGE);
	
	pinMode(SONAR2_ECHO,INPUT);
	pinMode(SONAR2_TRIG,OUTPUT);
	attachInterrupt(14, sonar2_handle, CHANGE);
	
	pinMode(SONAR3_ECHO,INPUT);
	pinMode(SONAR3_TRIG,OUTPUT);
	attachInterrupt(10, sonar3_handle, CHANGE);
	
	pinMode(SONAR4_ECHO,INPUT);
	pinMode(SONAR4_TRIG,OUTPUT);
	attachInterrupt(8, sonar4_handle, CHANGE);
	
	pinMode(SONAR5_ECHO,INPUT);
	pinMode(SONAR5_TRIG,OUTPUT);
	attachInterrupt(4, sonar5_handle, CHANGE);
	
	pinMode(SONAR6_ECHO,INPUT);
	pinMode(SONAR6_TRIG,OUTPUT);
	attachInterrupt(5, sonar6_handle, CHANGE);
	
	pinMode(SONAR7_ECHO,INPUT);
	pinMode(SONAR7_TRIG,OUTPUT);
	attachInterrupt(6, sonar7_handle, CHANGE);
	
	pinMode(SONAR8_ECHO,INPUT);
	pinMode(SONAR8_TRIG,OUTPUT);
	attachInterrupt(7, sonar8_handle, CHANGE);
	
	//attachInterrupt(3, sonar1_handle, FALLING);
	this->serial = serial;
}

void CSonar::update(void)
{
	if(isRunning){
		return;
	}
	clear();
	isRunning = true;
	digitalWrite(SONAR1_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR2_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR3_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR4_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR5_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR6_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR7_TRIG,GPIO_PIN_SET);
	digitalWrite(SONAR8_TRIG,GPIO_PIN_SET);
	delay_us(50);
	digitalWrite(SONAR1_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR2_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR3_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR4_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR5_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR6_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR7_TRIG,GPIO_PIN_RESET);
	digitalWrite(SONAR8_TRIG,GPIO_PIN_RESET);
	
	LED_TOGGLE(3);
}

void CSonar::upload(void)
{
	isRunning = false;
	
	sonar[0].distance = sonar[0].distanceData*0.000173f;
	sonar[1].distance = sonar[1].distanceData*0.000173f;
	sonar[2].distance = sonar[2].distanceData*0.000173f;
	sonar[3].distance = sonar[3].distanceData*0.000173f;
	sonar[4].distance = sonar[4].distanceData*0.000173f;
	sonar[5].distance = sonar[5].distanceData*0.000173f;
	sonar[6].distance = sonar[6].distanceData*0.000173f;
	sonar[7].distance = sonar[7].distanceData*0.000173f;
	
#if 0
	sprintf(send_buf,"sonar:%f %f %f %f %f %f %f %f\n"
	,sonar[0].distance
	,sonar[1].distance
	,sonar[2].distance
	,sonar[3].distance
	,sonar[4].distance
	,sonar[5].distance
	,sonar[6].distance
	,sonar[7].distance
	);
	this->serial->write((uint8_t*)send_buf,strlen(send_buf));
#else
	send_buf[0] = 0x40;
	send_buf[1] = 0x0F;
	send_buf[2] = 0x50;
	memcpy(&send_buf[3],&sonar[0].distance,sizeof(float));
	memcpy(&send_buf[7],&sonar[1].distance,sizeof(float));
	memcpy(&send_buf[11],&sonar[2].distance,sizeof(float));
	memcpy(&send_buf[15],&sonar[3].distance,sizeof(float));
	
	this->serial->write((uint8_t*)send_buf,send_buf[1]+3);
#endif
}

void CSonar::clear(){
	sonar[0].distanceData = 0;
	sonar[0].distance = 0.0f;
	
	sonar[1].distanceData = 0;
	sonar[1].distance = 0.0f;
	
	sonar[2].distanceData = 0;
	sonar[2].distance = 0.0f;
	
	sonar[3].distanceData = 0;
	sonar[3].distance = 0.0f;
	
	sonar[4].distanceData = 0;
	sonar[4].distance = 0.0f;
	
	sonar[5].distanceData = 0;
	sonar[5].distance = 0.0f;
	
	sonar[6].distanceData = 0;
	sonar[6].distance = 0.0f;
	
	sonar[7].distanceData = 0;
	sonar[7].distance = 0.0f;
}
