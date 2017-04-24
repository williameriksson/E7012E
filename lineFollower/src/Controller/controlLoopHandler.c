#include "controlLoopHandler.h"

PID motorPID, steeringPID;
const int looptime = 50; //controller loop in ms

void initControlLoopHandler() {
	//Controller and timer below
	__disable_irq();
	DistanceTemp = 0.0f;
	AdjustTemp = 0.0f;

	initController(&motorPID, 0.5f, 2.5f, 0.00f, 0.0f, looptime); //enables PID for motor
	initController(&steeringPID, 0.0f, 5.0f, 1.0f, 20.0f, looptime); //enables PID for steering

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enables TIM4 timer
	TIM4->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM4->PSC = 10000-1; //sets prescalar -> clock freq 10 kHz
	TIM4->ARR = (looptime*10)-1; //10Hz freq
	TIM4->CR1 |= TIM_CR1_CEN; //enables the Timer.

	__enable_irq();

	NVIC_EnableIRQ(TIM4_IRQn); //enables the interrupt
	NVIC_SetPriority(TIM4_IRQn, 22);

}

void setSpeed(float mps) {
	changeReference(&motorPID, mps);
	resetPIDError(&motorPID);
}

void runMotorControl() {
	float adjustment = runController(&motorPID, speed);
	adjustMotorPWM(adjustment);
}
int previousDistance = 0;
void runSteeringControl() {
	float distanceFromLine = getDistanceOffset();
	if(distanceFromLine > (float)LINESENSORARRAY_SIZE/2) {
		//has driven off the line.
		distanceFromLine = previousDistance/abs(previousDistance) * 5.0f;
	}
	else {
		previousDistance = distanceFromLine;
	}
	float adjustment = runController(&steeringPID, distanceFromLine);
	DistanceTemp = distanceFromLine;
	AdjustTemp = adjustment;
	adjustSteeringPWM(adjustment);
}

void TIM4_IRQHandler(void) {
	TIM4->SR &= ~0x1;
	runMotorControl();
	runSteeringControl();
}