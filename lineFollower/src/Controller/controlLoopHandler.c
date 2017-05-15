#include "controlLoopHandler.h"

const int looptimeMotor = 50; //controller loop in ms
const int looptimeSteering = 20;

int enableController;

void initControlLoopHandler() {
	//Controller and timer below
	__disable_irq();

	enableController = 1;
	magnetTickThreshhold = 20;
	totalAdjustment = 0.0f;
	FFmodifier = 6.14;
	obstacleToggle = 0;
	initController(&motorPID, 0.5f, 2.0f, 0.01f, 2.0f, looptimeMotor, 0.75f); //enables PID for motor
	initController(&steeringPID, 0.0f, 3.0f, 0.2f, 100.0f, looptimeSteering, 0.8f); //enables PID for steering

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enables TIM4 timer
	TIM4->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM4->PSC = 10000-1; //sets prescaler -> clock freq 10 kHz
	TIM4->ARR = (looptimeMotor*10)-1; //10Hz freq
	TIM4->CR1 |= TIM_CR1_CEN; //enables the Timer.

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM5->PSC = 10000-1; //sets prescaler -> clock freq 10 kHz
	TIM5->ARR = (looptimeSteering*10)-1; //10Hz freq
	TIM5->CR1 |= TIM_CR1_CEN; //enables the Timer

	__enable_irq();

	NVIC_EnableIRQ(TIM4_IRQn); //enables the interrupt
	NVIC_SetPriority(TIM4_IRQn, 22);
	NVIC_EnableIRQ(TIM5_IRQn); //enables the interrupt
	NVIC_SetPriority(TIM5_IRQn, 22);
}

void stopControllers() {
	enableController = 0;
	resetMotorPWM();
}

void startControllers() {
	resetPIDError(&motorPID);
	resetPIDError(&steeringPID);
	enableController = 1;
}

void setSpeed(float mps) {
	changeReference(&motorPID, mps);
	resetPIDError(&motorPID);
}

void setObstacleAvoidanceTimer(float newTime) {
	magnetTickThreshhold = (int)newTime;
}

float getObstacleAvoidanceTimer() {
	return (float)magnetTickThreshhold;
}

void setFeedforwardConstant(float value) {
	FFmodifier = value;
}

float getFeedforwardConstant() {
	return FFmodifier;
}

void runMotorControl() {
	float adjustment = runController(&motorPID, speed, 0);
	adjustMotorPWM(adjustment);
}

void toggleObstacleSensor() {
	obstacleToggle = !obstacleToggle;
}

float adjustTemp;
float feedForwardTemp;
int previousDistance = 0;
void runSteeringControl() {
	static int obstacleDetected = 0;
	float distanceFromLine = getDistanceOffset();
	if(distanceFromLine > (float)LINESENSORARRAY_SIZE/2) {
		//has driven off the line.
		distanceFromLine = previousDistance/abs(previousDistance) * 10.0f;
	}
	else {
		previousDistance = distanceFromLine;
	}

	float feedForwardAngle = (-1.0f) * FFmodifier * distanceFromLine; // Angle in degrees
	int feedForwardAdjustment = setSteering(feedForwardAngle); // Get the PW value difference
	float adjustment = runController(&steeringPID, distanceFromLine, isSaturatedSteering);

	if(obstacleToggle) {
		if (obstacleDistance <= obstacleThreshold && !obstacleDetected && traveledDistance >= 0.5f) {
			magnetTick = 0;
			obstacleDetected = 1;
		}

		if (obstacleDetected) {
			if (magnetTick <= magnetTickThreshhold) {
				adjustment = 15000;
				feedForwardAdjustment = 0.0f;
			}
	//		else if (magnetTick <= 26) {
	//			adjustment = -15000;
	//		}
			else {
				obstacleDetected = 0;
			}
		}
	}

	adjustTemp = adjustment;
	feedForwardTemp = (float)feedForwardAdjustment;
	totalAdjustment = adjustment + (float)feedForwardAdjustment;
	adjustSteeringPWM(totalAdjustment);
	//adjustSteeringPWM(adjustment);
	//adjustSteeringPWM((float)feedForwardAdjustment);
}

void TIM4_IRQHandler(void) {
	TIM4->SR &= ~0x1;
	if(enableController) {
		runMotorControl();
	}
}

void TIM5_IRQHandler() {
	TIM5->SR &= ~0x1;
	if(enableController) {
		runSteeringControl();
	}
}

