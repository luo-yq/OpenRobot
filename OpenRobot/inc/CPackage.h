#ifndef _CPACKAGE_H_
#define _CPACKAGE_H_

#include <string.h>
#include "UARTClass.h"
#include "drv_uart.h"

//frame type
typedef enum{
  MSG_ODOM = 1,
  MSG_SPEED = 2,
  MSG_BATTERY = 3,
  MSG_IMU = 4,
  MSG_WHEEL  = 5,
  MSG_MOVECTRL = 0x81,
  MSG_SERVOCTRL = 0x82,
	MSG_LOG = 0x97,
  MSG_OK = 0x98,
  MSG_TEST = 0x99
}msg_type;

#ifndef BTYE
#define BYTE unsigned char
#endif

//frame head
#define FRAME_HEAD_1    0x55
#define FRAME_HEAD_2    0xAA
//frame end
#define FRAME_END       0x0A


typedef struct
{

int count1;
int count2;
int count3;

float speed1;
float speed2;
float speed3;

}ODOM_MSG;

typedef struct{
  int16_t acc[3];
  int16_t gyro[3];

  float w;
  float x;
  float y;
  float z;
}IMU_MSG;

class robot_serial
{
	public:
		robot_serial();
		~robot_serial();
		void Init();
		void Updata();
		void Send_Package(void* data, msg_type type, int len);
		bool isRecv(unsigned char *buffer);
		static bool isOK;
		//bool isRecv;
		uint8_t serialBuffer[100];
};


#endif

