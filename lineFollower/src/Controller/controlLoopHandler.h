#ifndef CONTROLLER_CONTROLLOOPHANDLER_H_
#define CONTROLLER_CONTROLLOOPHANDLER_H_

#include "stm32f4xx.h"
#include "../SteeringControl/steeringControl.h"
#include "../MotorControl/motorControl.h"
#include "../LineSensor/lineSensorReader.h"
#include "math.h"

float DistanceTemp;
float AdjustTemp;

void initControlLoopHandler(void);
void setSpeed(float);
void runMotorControl(void);
void runSteeringControl(void);

#endif /* CONTROLLER_CONTROLLOOPHANDLER_H_ */