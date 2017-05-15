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

PID motorPID;
PID steeringPID;
int magnetTickThreshhold;
float totalAdjustment;
float FFmodifier;
int obstacleToggle;

void initControlLoopHandler(void);
void stopControllers();
void startControllers();
void setSpeed(float);
void runMotorControl(void);
void runSteeringControl(void);
void setObstacleAvoidanceTimer(float);
float getObstacleAvoidanceTimer();
void setFeedforwardConstant(float);
float getFeedforwardConstant();
void toggleObstacleSensor();

#endif /* CONTROLLER_CONTROLLOOPHANDLER_H_ */
