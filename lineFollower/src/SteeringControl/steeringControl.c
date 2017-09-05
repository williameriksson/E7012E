#include "steeringControl.h"

const int steeringminPW = 1100; //1.3 ms pulse width
const int steeringneutralPW = 1500; //1.5ms pulse width
const int steeringmaxPW = 1900; //1.7 ms pulse width
const float maxSteeringAngle = 30.0; //the angular span between center and max right/left steering. TODO:figure this out

void initSteeringControl() {
	//Init for servo control on TIM 3 Ch 4 (PC9)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //ensures clock on GPIOC is enabled
	GPIOC->MODER |= GPIO_MODER_MODER9_1; //sets GPIOC mode to alternating function
	GPIOC->AFR[1] |= (((uint8_t)0x02) << 4);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enables TIM3 timer
	TIM3->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM3->PSC = 100-1; //sets prescalar -> clock freq 100 kHz
	TIM3->ARR = 20000-1;
	TIM3->CCR4 = 20000 - 1500 - 1; // CCR4 timer (for servo this determines angle (1-2 ms pulse width))
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_0;
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1; //sets CCMR2 to mode 2... 111
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM3->CCER |= TIM_CCER_CC4E; //capture/compare ch4 enabled

	TIM3->CR1 |= TIM_CR1_CEN; //Start timer
	__enable_irq();

	isSaturatedSteering = 0;
}
void adjustSteeringPWM(float amount) {
	int pwAdjust = (int)amount;
	int currentPW = 20000 - TIM3->CCR4 - 1;

	if(currentPW + pwAdjust < steeringminPW) {
		TIM3->CCR4 = 20000 - steeringminPW - 1;
		isSaturatedSteering = 1;
	}
	else if(currentPW + pwAdjust > steeringmaxPW) {
		TIM3->CCR4 = 20000 - steeringmaxPW - 1;
		isSaturatedSteering = 1;
	}
	else {
		TIM3->CCR4 = 20000 - (currentPW + pwAdjust) - 1;
		isSaturatedSteering = 0;
	}
}

int setSteering(float angle) {
	int pulseWidth;
	if(angle <= -maxSteeringAngle) {
		pulseWidth = steeringminPW;
	}
	else if(angle >= maxSteeringAngle) {
		pulseWidth = steeringmaxPW;
	}
	else {
		//sets the PWM signal to same proportions as angle is to maxSteeringAngle.
		float turnPercentage = angle / maxSteeringAngle;
		int spanPW = (steeringmaxPW - steeringminPW) / 2;
		pulseWidth = steeringneutralPW + (int)(spanPW * turnPercentage);
	}
	//TIM3->CCR4 = 20000 - pulseWidth - 1;
	return (TIM3->CCR4 - (20000 - pulseWidth - 1));
}
