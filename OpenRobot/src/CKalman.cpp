#include "CKalman.h"

CKalman::CKalman()
{

	this->MeasureNoise_R = 0.002;
	this->ProcessNiose_Q = 0.02;
	this->X_Last = 0;
	this->P_Last = 0;
}

CKalman::~CKalman()
{

}


float CKalman::update(float z)
{
	double X_Mid;
	double P_Mid;
	double P_Now;
	double X_Now;
	double kg;

	X_Mid = this->X_Last;
	P_Mid = this->P_Last+this->ProcessNiose_Q;

	kg = P_Mid/(P_Mid+this->MeasureNoise_R);

	X_Now = X_Mid+kg*(z-X_Mid);
	P_Now = (1-kg)*P_Mid;

	this->X_Last = X_Now;
	this->P_Last = P_Now;

	return (float)X_Now;
	
}


