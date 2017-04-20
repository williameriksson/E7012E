#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "stm32f4xx.h"
#include "Controller/PIDcontroller.h"
#include "HallSensor/hallSensorReader.h"

void initMotorControl(void);
void accelerate(float);

#endif /* MOTORCONTROL_H_ */
