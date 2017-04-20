#include "lineSensorReader.h"

//linesensor takes input on port PB0 - PB8.
LineSensorArray lsr;


void initLineSensorReader() {
	__disable_irq();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enables clock on GPIOB
	GPIOB->PUPDR &= 0x0; //disables push-pull resistors.
	GPIOB->MODER &= ~(0x3FFFF); //sets MODER 0-8 to 00 (input mode) (sets PB0-PB8 for GPIO input mode)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables the global interrupts on TIM2 (on counter overflow)
	TIM2->PSC = 1000-1; //sets prescalar -> clock freq 100 kHz
	TIM2->ARR = 1000-1; //100Hz freq
	TIM2->CR1 |= TIM_CR1_CEN; //enables the Timer.
	__enable_irq();

	NVIC_EnableIRQ(TIM2_IRQn); //enables the interrupt
	NVIC_SetPriority(TIM2_IRQn, 22);
}

void updateLineSensorArray() {
	int sensorArrayStatusBits = (GPIOB->IDR & 0x1FF); //applies bitmask for bit 0-8 of Input data register.
	for(int i = 0; i < LINESENSORARRAY_SIZE; i++) {
			lsr.values[i] = ((sensorArrayStatusBits & (int)pow((double)2, (double)i)) >> i);
	}
}

float getDistanceOffset() {
	int maxSensorDistanceOffset = LINESENSORARRAY_SIZE / 2;
	int distanceFromCenter = 0;
	int activeSensorsCount = 0;
	for(int i = 0; i < LINESENSORARRAY_SIZE; i++) {
		if(lsr.values[i] == 1) {
			distanceFromCenter += maxSensorDistanceOffset - i;
			activeSensorsCount++;
		}
	}
	if(activeSensorsCount != 0) {
		return (float)distanceFromCenter / (float)activeSensorsCount;
	}
	else {
		return 0.0f;
	}
}

void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
	updateLineSensorArray();
}

