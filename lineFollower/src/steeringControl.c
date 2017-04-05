#include "steeringControl.h"

const int minPW = 12000; //1 ms pulse width
const int neutralPW = 15000; //1.5ms pulse width
const int maxPW = 18000; //2 ms pulse width
const float maxSteeringAngle = 45.0; //the angular span between center and max right/left steering. TODO:figure this out

void initServoControl() {
	//Init for servo control on pin PB3 (TIM2 ch2)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOB is enabled
	GPIOB->MODER |= GPIO_MODER_MODER3_1; //sets GPIOB mode to alternating function
	GPIOB->AFR[0] |= (((uint8_t)0x01) << 12);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10-1; //sets prescalar -> clock freq 1MHz
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
	if(angle <= -maxSteeringAngle) {
		TIM2->CCR2 = 20000 - minPW - 1;
	}
	else if(angle >= maxSteeringAngle) {
		TIM2->CCR2 = 20000 - maxPW - 1;
	}
	else {
		//sets the PWM signal to same proportions as angle is to maxSteeringAngle.
		float turnPercentage = angle / maxSteeringAngle;
		int spanPW = (maxPW - minPW) / 2;
		int turnPW = neutralPW + (int)(spanPW * turnPercentage);
		TIM2->CCR2 = 20000 - turnPW - 1;
	}
}
