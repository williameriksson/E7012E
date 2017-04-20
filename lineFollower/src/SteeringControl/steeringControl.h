#ifndef STEERINGCONTROL_H_
#define STEERINGCONTROL_H_

#include "stm32f4xx.h"

void initSteeringControl(void);
void setSteering(float);
void runSteeringControl(void);

#endif /* STEERINGCONTROL_H_ */
