#include "motorControl.h"

const int minPW = 1000; //1 ms pulse width
const int neutralPW = 1500; //1.5ms pulse width
const int maxPW = 2000; //2 ms pulse width
const float maxSpeed = 8.3f;

void initMotorControl() {
	//TODO : switch to TIM 3 Ch 3 (PC8)
	//Init for motor control on pin PB10 (TIM2 ch3)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //ensures clock on GPIOC is enabled
	GPIOC->MODER |= GPIO_MODER_MODER8_1; //sets GPIOC mode to alternating function
	GPIOC->AFR[1] |= (((uint8_t)0x02) << 0);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enables TIM3 timer
	TIM3->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM3->PSC = 100-1; //sets prescalar -> clock freq 100 kHz
	TIM3->ARR = 20000-1; //50Hz freq (how often the pulses will arrive)
	TIM3->CCR3 = 20000 - 1500 - 1; // CCR1 timer (for motor this determines speed (1000-2000))
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_0;
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1; //sets CCRM2 OC3M to mode 2... 111
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM3->CCER |= TIM_CCER_CC3E; //capture/compare ch3 enabled
	TIM3->CR1 |= TIM_CR1_CEN; //enables the Timer.
	__enable_irq();
}
/*
 * Sets the pulsewidth of PWM controlling the motor to be same proportions as mps is to maxSpeed
 */
void setSpeed(float mps) {
	int pulseWidth;
	if(mps >= maxSpeed) {
		pulseWidth = minPW;
	}
	else {
		float speedPercentage = mps / maxSpeed;
		int spanPW = (maxPW - neutralPW);
		pulseWidth = neutralPW - (int)(spanPW * speedPercentage);
	}
	TIM3->CCR3 = 20000 - pulseWidth - 1;
}
