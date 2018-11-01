#include "CMotor.h"

#define KP  2.2
#define KI  0.36
#define KD  0.8
#define ROTATE_SPEED  6.0606



// right
#define MOTOR1_A_CH MOTOR_CH3
#define MOTOR1_B_CH MOTOR_CH4
#define ENCODER1_A  EXTI_PIN_2
#define ENCODER1_B  EXTI_PIN_3

// back
#define MOTOR2_A_CH MOTOR_CH5
#define MOTOR2_B_CH MOTOR_CH6
#define ENCODER2_A  EXTI_PIN_4
#define ENCODER2_B  EXTI_PIN_5

// left
#define MOTOR3_A_CH MOTOR_CH7
#define MOTOR3_B_CH MOTOR_CH8
#define ENCODER3_A  EXTI_PIN_6
#define ENCODER3_B  EXTI_PIN_7




#define EXTI_PIN_0 80
#define EXTI_PIN_1 81
#define EXTI_PIN_2 82
#define EXTI_PIN_3 83
#define EXTI_PIN_4 84
#define EXTI_PIN_5 85
#define EXTI_PIN_6 86
#define EXTI_PIN_7 87
//the pin number of encoder, ENCODERX is for motorX




extern "C"{
void Count_and_Direction(Wheel *omni) {

#if 1
	int ss = digitalRead(omni->encoder_a);

	//rising
	if(ss == HIGH)
	{
			
		    omni->count += 1;
		    if (digitalRead(omni->encoder_b) == LOW) {
		      omni->dir = 0; //forward
		      omni->counter ++;
		    }
		    else {
		      omni->dir = 1; //backward
		      omni->counter --;
		    }	
	}

#else
  omni->count += 1;
		
  if (digitalRead(omni->encoder_a) == HIGH) {
    if (digitalRead(omni->encoder_b) == LOW) {
      omni->dir = 0; //forward
      omni->counter ++;
    }
    else {
      omni->dir = 1; //backward
      omni->counter -= 1;
    }
  }
  else if (digitalRead(omni->encoder_b) == HIGH) {
    omni->dir = 0; //forward
    omni->counter ++;
  }
  else {
    omni->dir = 1; //backward
    omni->counter -= 1;
  }
#endif
  // Serial.println(omni->count);
}
}
float velocity_calculate(Wheel *omni) {
  float ang;
  ang = float(omni->count * 1.0);
  if (omni->dir == 1) {
    ang = -ang;
  }
  omni->count = 0;
  return ang;
}

void PID_controller(MOTOR_PID *pid)
{
  if(abs(pid->target) <= 200)// && abs(pid->target) >= 0)
  {
    double out;  
    pid->Proportion = abs(pid->target) - pid->feedback;
    pid->Integral += (pid->Ki*pid->Proportion);
    pid->Integral = constrain(pid->Integral,0,200);
    pid->Differential = (pid->feedback - pid->last_feedback);

    out = pid->Kp*pid->Proportion + pid->Integral - pid->Kd*pid->Differential; 
	//pid->output = out;
    
    if (out - pid->output > 30) {
      out = pid->output + 30;
    }
    //pid->output = 50.0;
/*    if(pid->output > 120.0)
    {
    	pid->output = 120.0;
    }

    if(pid->output <= 1.0)
    {
    	pid->output = 0.0;
    }*/
		pid->output = constrain(out,0,160);
    pid->last_feedback = pid->feedback;
    if(abs(pid->target) >= 1)
    {
      if (pid->target <= 0) 
      {
        drv_pwm_set_duty(pid->motorB, 0);
        drv_pwm_set_duty(pid->motorA, (uint8_t)abs(pid->output));
      }
      else 
      {
        drv_pwm_set_duty(pid->motorA, 0);
        drv_pwm_set_duty(pid->motorB, (uint8_t)abs(pid->output));
      }
    }
    else
    {
      drv_pwm_set_duty(pid->motorA, 256);
      drv_pwm_set_duty(pid->motorB, 256);    
    }
  }
  else
  {
      drv_pwm_set_duty(pid->motorA, 256);
      drv_pwm_set_duty(pid->motorB, 256);
  }
}


CMotor::CMotor()
{
  /******************************************************************************
   PID Init
  ******************************************************************************/
  // left wheel initialize pid parameter
  m_pid[0].e_0 = 0;
  m_pid[0].e_1 = 0;
  m_pid[0].e_2 = 0;
  m_pid[0].Proportion = 0;
  m_pid[0].Integral = 0.0;
  m_pid[0].Differential = 0;
  m_pid[0].Kp = KP;
  m_pid[0].Ki = KI;
  m_pid[0].Kd = KD;
  m_pid[0].output = 0;
  m_pid[0].target = 0;
  m_pid[0].feedback = 0;
  m_pid[0].last_feedback = 0;
  m_pid[0].motorA = MOTOR1_A_CH;
  m_pid[0].motorB = MOTOR1_B_CH;
  
  // right wheel initialize pid parameter
  m_pid[1].e_0 = 0;
  m_pid[1].e_1 = 0;
  m_pid[1].e_2 = 0;
  m_pid[1].Proportion = 0;
  m_pid[1].Integral = 0.0;
  m_pid[1].Differential = 0;
  m_pid[1].Kp = KP;
  m_pid[1].Ki = KI;
  m_pid[1].Kd = KD;
  m_pid[1].output = 0;
  m_pid[1].target = 0;
  m_pid[1].feedback = 0;
  m_pid[1].last_feedback = 0;
  m_pid[1].motorA = MOTOR2_A_CH;
  m_pid[1].motorB = MOTOR2_B_CH;

  m_pid[2].e_0 = 0;
  m_pid[2].e_1 = 0;
  m_pid[2].e_2 = 0;
  m_pid[2].Proportion = 0;
  m_pid[2].Integral = 0.0;
  m_pid[2].Differential = 0;
  m_pid[2].Kp = KP;
  m_pid[2].Ki = KI;
  m_pid[2].Kd = KD;
  m_pid[2].output = 0;
  m_pid[2].target = 0;
  m_pid[2].feedback = 0;
  m_pid[2].last_feedback = 0;
  m_pid[2].motorA = MOTOR3_A_CH;
  m_pid[2].motorB = MOTOR3_B_CH;



  omni_wheel[0].encoder_a = ENCODER1_A;
  omni_wheel[0].encoder_b = ENCODER1_B;
  omni_wheel[0].counter = 0;
  omni_wheel[0].count = 0;
  omni_wheel[0].dir = 0;

  omni_wheel[1].encoder_a = ENCODER2_A;
  omni_wheel[1].encoder_b = ENCODER2_B;
  omni_wheel[1].counter = 0;
  omni_wheel[1].count = 0;
  omni_wheel[1].dir = 0;

  omni_wheel[2].encoder_a = ENCODER3_A;
  omni_wheel[2].encoder_b = ENCODER3_B;
  omni_wheel[2].counter = 0;
  omni_wheel[2].count = 0;
  omni_wheel[2].dir = 0;
}

CMotor::~CMotor()
{
}

void CMotor::Init()
{
}

void CMotor::controller()
{
	  PID_controller(&m_pid[0]);
    PID_controller(&m_pid[1]);
    PID_controller(&m_pid[2]);
}

bool CMotor::readEncoder(int &s1,int &s2,int &s3)
{
	__disable_irq();
	s1 = omni_wheel[0].counter;
	s2 = omni_wheel[1].counter;
	s3 = omni_wheel[2].counter;
	__enable_irq();
	return true;
}

void CMotor::setSpeed(double s1,double s2,double s3)
{
	m_pid[0].target = s1;
  m_pid[1].target = s2;
  m_pid[2].target = s3;
}
void CMotor::FeedbackSpeed(double s1,double s2,double s3)
{
	m_pid[0].feedback = abs(s1);
	m_pid[1].feedback = abs(s2);
	m_pid[2].feedback = abs(s3);

}




