#ifndef _IMU_H_
#define _IMU_H_

//#include <inttypes.h>
//#include <Arduino.h>
#include "SPI.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"


#define IMU_OK			  0x00
#define IMU_ERR_I2C		0x01










class cIMU
{

public:
	cMPU9250 SEN;


	int16_t angle[3];
  float   rpy[3];
  float   quat[4];
  int16_t gyroData[3];
  int16_t gyroRaw[3];
  int16_t accData[3];
  int16_t accRaw[3];
  int16_t magData[3];
  int16_t magRaw[3];

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

	bool bConnected;

  float aRes;
  float gRes;
  float mRes;
  Madgwick filter;
public:
	cIMU();

	uint8_t  begin( uint32_t hz = 200 );
	uint16_t update( uint32_t option = 0 );
  
private:
  
  uint32_t update_hz;
  uint32_t update_us;

	void computeIMU( void );

};


#endif
