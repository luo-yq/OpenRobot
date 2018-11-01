#ifndef _CKALMAN_H
#define _CKALMAN_H
#include <string.h>
#include "drv_include.h"

class CKalman
{

public:
	CKalman();
	~CKalman();
	float update(float z);


	void setProcessNiose_Q(double Q){
		this->ProcessNiose_Q = Q;
	}
	void setMeasureNoise_R(double R){
		this->MeasureNoise_R = R;
	}
	double ProcessNiose_Q;
	double MeasureNoise_R;//
	double X_Last;
	double P_Last;
	int num;
};


#endif

