#ifndef CONTROLLER_CONTROLLOOPHANDLER_H_
#define CONTROLLER_CONTROLLOOPHANDLER_H_

#include "stm32f4xx.h"
#include "../SteeringControl/steeringControl.h"
#include "../MotorControl/motorControl.h"
#include "../LineSensor/lineSensorReader.h"
#include "../HallSensor/hallSensorReader.h"
#include "../ObstacleSensor/obstacleSensor.h"
#include "PIDcontroller.h"
#include <math.h>

float DistanceTemp;
float AdjustTemp;
PID motorPID;
PID steeringPID;

void initControlLoopHandler(void);
void stopControllers();
void startControllers();
void setSpeed(float);
void runMotorControl(void);
void runSteeringControl(void);

#endif /* CONTROLLER_CONTROLLOOPHANDLER_H_ */
