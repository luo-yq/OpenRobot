#include "CPackage.h"
#include "drv_led.h"
static BYTE readBuffer[50];
static uint32_t readLen = 0;
static int match_flag = 0;
static BYTE inChar;
static BYTE match[2];
static int serial_cnt = 0; //global
bool robot_serial::isOK = false;
extern UARTClass Serial1;


static bool checkSum(unsigned char* buf)
{
  unsigned char sum = 0;
  unsigned short len = 0;
  len = buf[3]*256 + buf[4];
  
  for(int i = 5; i< len+5; i++)
  {
    sum+=buf[i];
  }
  
  if(sum != buf[len+5])
  {
    return false;
  }
  //ROS_INFO("%02X",buf[2]);
  return true;
}






robot_serial::robot_serial()
{
	memset(this->serialBuffer,0,sizeof(this->serialBuffer));

	//isRecv = false;
}

robot_serial::~robot_serial()
{

}



void robot_serial::Updata() {
	uint32_t len = Serial1.available();
    if (len>10) {
	 for(int i = 0;i<len;i++)
	 {
	 	inChar = (unsigned char)Serial1.read();
	 	   if (match_flag == 0) {
		      match[0] = inChar;
		      if (match[0] == 0x55) {
		        match_flag = 1;
		        readBuffer[0] = 0x55;
		        serial_cnt ++;
		        continue;
		      }
		      else {
		        match_flag = 0;
		        serial_cnt = 0;
		        continue;
		      }
		    }
		    if (match_flag == 1) {
		      match[1] = inChar;
		      if (match[1] == 0xAA) {
		        match_flag = 2;
		        readBuffer[1] = 0xAA;
		        serial_cnt ++;
		        continue;
		      }
		      else {
		        match_flag = 0;
		        serial_cnt = 0;
		        continue;
		      }
		    }
		    if (match_flag == 2) {
		      inChar = inChar;
		      readBuffer[serial_cnt] = inChar;
		      serial_cnt ++;
		      match_flag = 3;
		      continue;
		    }
		    if (match_flag == 3) {
		      inChar = inChar;
		      readBuffer[serial_cnt] = inChar;
		      serial_cnt ++;
		      readLen = readBuffer[3]*256 + readBuffer[4];
		      match_flag = 4;
		      continue;
		    }
		    if (match_flag == 4) {
		      
		      readBuffer[serial_cnt] = inChar;
		      serial_cnt ++;
		      if ((readLen < 60 && serial_cnt >= (readLen+5+4))) {
		        if(checkSum(readBuffer) && robot_serial::isOK == false)
		        {
		          robot_serial::isOK = true;
		          
		          match_flag = 5;
		          return;          
		        }
		        else
		        {
		          serial_cnt = 0;
		          match_flag = 0; 
			   continue;
		        }

		      }
		    }
	 }
 
  }
}


void robot_serial::Init()
{
  Serial1.begin((uint32_t)57600);
  match_flag = 0;
}

void robot_serial::Send_Package(void* data, msg_type type, int len)
{
	BYTE a = 0;
	BYTE buffer[256];
  unsigned short slen = len;
	buffer[0] = FRAME_HEAD_1;
	buffer[1] = FRAME_HEAD_2;
	buffer[2] = type;
	buffer[3] = len/256;
  buffer[4] = len%256;
	memcpy((uint8_t*)&buffer[5], data, slen);
	for (int i = 5; i < (slen+5); i++) {
	  a += buffer[i];
	}
	buffer[slen+5] = a;
	a = 0;
	buffer[slen+6] = 0;
	buffer[slen+7] = 0;
	buffer[slen+8] = FRAME_END;
	Serial1.write((uint8_t*)buffer, (slen+9));
}

bool robot_serial::isRecv(unsigned char *buffer)
{
	if(this->isOK==true)
	{
		memcpy((uint8_t*)buffer, readBuffer, (readLen+5+3));
		this->isOK = false;
		serial_cnt = 0;
    match_flag = 0;
    //watch_dog = millis();
		return true;
	}
	return false;
}




