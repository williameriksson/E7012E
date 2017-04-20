#include "lineSensorReader.h"

//linesensor takes input on port PB0 - PB8.
LineSensorArray lsr;


void initLineSensorReader() {
	GPIOB->MODER &= ~(0x3FFFF); //sets MODER 0-8 to 00 (input mode) (sets PB0-PB8 for GPIO input mode)

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10000-1; //sets prescalar -> clock freq 10 kHz
	TIM2->ARR = 1000-1; //100Hz freq
	TIM2->DIER |= TIM_DIER_UIE; //enables the global interrupts on TIM2 (on counter overflow)
	TIM2->CR1 |= TIM_CR1_CEN; //enables the Timer.
}

void updateLineSensorArray() {
	int sensorArrayStatusBits = (GPIOB->IDR & 0x1FF); //applies bitmask for bit 0-8 of Input data register.
	for(int i = 0; i < LINESENSORARRAY_SIZE; i++) {
		lsr.values[i] = sensorArrayStatusBits | (int)pow((double)2, (double)i);
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
	return (float)distanceFromCenter / (float)activeSensorsCount;
}

void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;
	updateLineSensorArray();
}

