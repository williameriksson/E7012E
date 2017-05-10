#ifndef STEERINGCONTROL_H_
#define STEERINGCONTROL_H_

#include "stm32f4xx.h"

void initSteeringControl(void);
int setSteering(float);
void adjustSteeringPWM(float);

int isSaturatedSteering;

#endif /* STEERINGCONTROL_H_ */
