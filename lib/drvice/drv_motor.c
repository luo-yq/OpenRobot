#include "drv_motor.h"
#include "WMath.h"


//the pin num of motor, two pins for 1 motor,  MOTOR1_A and MOTOR1_B are for the motor1
#define MOTOR1_A  60
#define MOTOR1_B  61
#define MOTOR2_A  62
#define MOTOR2_B  63
#define MOTOR3_A  73
#define MOTOR3_B  75
#define MOTOR4_A  77
#define MOTOR4_B  78

//static uint32_t motor_pin[8] = {60,61,62,63,73,75,77,78};

struct t_motor_pwm{
	TIM_HandleTypeDef hTIM_;
	TIM_OC_InitTypeDef hOC[4];
  uint32_t pwm_freq;
};

uint32_t tim_ch[6] = {TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_5,TIM_CHANNEL_6};


struct t_motor_pwm _pwm[2] = {0};


void drv_motor_pwm_init()
{
	_pwm[0].hTIM_.Instance = TIM4;
	_pwm[0].pwm_freq = 1000;

  _pwm[0].hTIM_.Init.Prescaler         = 8-1;
  _pwm[0].hTIM_.Init.Period            =216000000/8/_pwm[0].pwm_freq/2-1;
  _pwm[0].hTIM_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  _pwm[0].hTIM_.Init.CounterMode       = TIM_COUNTERMODE_UP;
  _pwm[0].hTIM_.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&_pwm[0].hTIM_);

	for(int i = 0; i<4; i++)
	{
	  _pwm[0].hOC[i].OCMode       = TIM_OCMODE_PWM1;
	  _pwm[0].hOC[i].OCPolarity   = TIM_OCPOLARITY_HIGH;
	  _pwm[0].hOC[i].OCFastMode   = TIM_OCFAST_DISABLE;
	  _pwm[0].hOC[i].OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	  _pwm[0].hOC[i].OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  _pwm[0].hOC[i].OCIdleState  = TIM_OCIDLESTATE_RESET;		
	}


	HAL_TIM_PWM_ConfigChannel(&_pwm[0].hTIM_, &_pwm[0].hOC[0], TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&_pwm[0].hTIM_, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&_pwm[0].hTIM_, &_pwm[0].hOC[1], TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&_pwm[0].hTIM_, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&_pwm[0].hTIM_, &_pwm[0].hOC[2], TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&_pwm[0].hTIM_, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&_pwm[0].hTIM_, &_pwm[0].hOC[3], TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&_pwm[0].hTIM_, TIM_CHANNEL_4);

	
	_pwm[1].hTIM_.Instance = TIM1;
	_pwm[1].pwm_freq = 1000;

  _pwm[1].hTIM_.Init.Prescaler         = 8-1;
  _pwm[1].hTIM_.Init.Period            = 216000000/8/_pwm[1].pwm_freq-1;
  _pwm[1].hTIM_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  _pwm[1].hTIM_.Init.CounterMode       = TIM_COUNTERMODE_UP;
  _pwm[1].hTIM_.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&_pwm[1].hTIM_);

	for(int i = 0; i<4; i++)
	{
	  _pwm[1].hOC[i].OCMode       = TIM_OCMODE_PWM1;
	  _pwm[1].hOC[i].OCPolarity   = TIM_OCPOLARITY_HIGH;
	  _pwm[1].hOC[i].OCFastMode   = TIM_OCFAST_DISABLE;
	  _pwm[1].hOC[i].OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	  _pwm[1].hOC[i].OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  _pwm[1].hOC[i].OCIdleState  = TIM_OCIDLESTATE_RESET;		
	}


	HAL_TIM_PWM_ConfigChannel(&_pwm[1].hTIM_, &_pwm[1].hOC[0], TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&_pwm[1].hTIM_, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&_pwm[1].hTIM_, &_pwm[1].hOC[1], TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&_pwm[1].hTIM_, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&_pwm[1].hTIM_, &_pwm[1].hOC[2], TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&_pwm[1].hTIM_, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&_pwm[1].hTIM_, &_pwm[1].hOC[3], TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&_pwm[1].hTIM_, TIM_CHANNEL_4);
	
}

void drv_pwm_set_duty(uint32_t ulPin, uint32_t ulDuty )
{
  TIM_HandleTypeDef  *pTIM;
  TIM_OC_InitTypeDef *pOC;
  
	uint8_t pwm_num = ulPin/4;
	uint8_t pwm_ch_num = ulPin%4;
	if(pwm_num >= 2 || pwm_ch_num>=4)return;

	pTIM = &_pwm[pwm_num].hTIM_;
	pOC = &_pwm[pwm_num].hOC[pwm_ch_num];
	
/*	if(ulDuty == 0) 
	{
		digitalWrite(motor_pin[ulPin],0);
		return;
	}
	if(ulDuty >= 250) 
	{
		digitalWrite(motor_pin[ulPin],1);
		return;
	}*/
	
  ulDuty = constrain(ulDuty, 1, (1<<(uint32_t)8)-1);
  pOC->Pulse = map( ulDuty, 0, (1<<(uint32_t)8)-1, 0, pTIM->Init.Period+1 );
//	TIM1->CCR4=pOC->Pulse; 
  HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch[pwm_ch_num]);
  HAL_TIM_PWM_Start(pTIM, tim_ch[pwm_ch_num]);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef	 GPIO_InitStruct;

  if( htim->Instance == TIM1 )
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  if( htim->Instance == TIM4 )
  {
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    GPIO_InitStruct.Pin       = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    GPIO_InitStruct.Pin       = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}


