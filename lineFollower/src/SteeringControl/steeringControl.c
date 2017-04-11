#include "steeringControl.h"

const int steeringminPW = 13000; //1.3 ms pulse width
const int steeringneutralPW = 15000; //1.5ms pulse width
const int steeringmaxPW = 17000; //1.7 ms pulse width
const float maxSteeringAngle = 30.0; //the angular span between center and max right/left steering. TODO:figure this out

void initServoControl() {
	//Init for servo control on pin PB3 (TIM2 ch2)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOB is enabled
	GPIOB->MODER |= GPIO_MODER_MODER3_1; //sets GPIOB mode to alternating function
	GPIOB->AFR[0] |= (((uint8_t)0x01) << 12);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10-1; //sets prescalar -> clock freq 10MHz
	TIM2->ARR = 200000-1;
	TIM2->CCR2 = 200000 - 15000 - 1; // CCR2 timer (for servo this determines angle (1-2 ms pulse width))
	TIM2->DIER |= TIM_DIER_CC2IE; //sets the CC1IE flag for interrupt enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_0;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1; //sets CCMR1 to mode 2... 111
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM2->CCER |= TIM_CCER_CC2E; //capture/compare ch2 enabled

	TIM2->CR1 |= TIM_CR1_CEN;
	__enable_irq();

}

void setSteering(float angle) {
	int pulseWidth;
	if(angle <= -maxSteeringAngle) {
//		TIM2->CCR2 = 200000 - steeringminPW - 1;
		pulseWidth = steeringminPW;
	}
	else if(angle >= maxSteeringAngle) {
//		TIM2->CCR2 = 200000 - steeringmaxPW - 1;
		pulseWidth = steeringmaxPW;
	}
	else {
		//sets the PWM signal to same proportions as angle is to maxSteeringAngle.
		float turnPercentage = angle / maxSteeringAngle;
		int spanPW = (steeringmaxPW - steeringminPW) / 2;
		pulseWidth = steeringneutralPW + (int)(spanPW * turnPercentage);
//		TIM2->CCR2 = 200000 - turnPW - 1;
	}
	TIM2->CCR2 = 200000 - pulseWidth - 1;
}
