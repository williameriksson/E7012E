#include "motorControl.h"

const int minPW = 10000; //1 ms pulse width
const int neutralPW = 15000; //1.5ms pulse width
const int maxPW = 20000; //2 ms pulse width
const float maxSpeed = 8.3f;

void initMotorControl() {
	//Init for motor control on pin PB10 (TIM2 ch3)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOB is enabled
	GPIOB->MODER |= GPIO_MODER_MODER10_1; //sets GPIOB mode to alternating function
	GPIOB->AFR[1] |= (((uint8_t)0x01) << 8);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10-1; //sets prescalar -> clock freq 1MHz
	TIM2->ARR = 200000-1; //50Hz freq (how often the pulses will arrive)
	TIM2->CCR3 = 200000 - 15000 - 1; // CCR1 timer (for motor this determines speed (1000-2000))
	TIM2->DIER |= TIM_DIER_CC2IE; //sets the CC2IE flag for interrupt
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_0;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1; //sets CCRM2 to mode 2... 111
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM2->CCER |= TIM_CCER_CC3E; //capture/compare ch3 enabled

	TIM2->CR1 |= TIM_CR1_CEN;
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
	TIM2->CCR3 = 200000 - pulseWidth - 1;
}
