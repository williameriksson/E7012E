#ifndef STEERINGCONTROL_H_
#define STEERINGCONTROL_H_

#include "stm32f4xx.h"

void initSteeringControl(void);
void setSteering(float);
void adjustSteeringPWM(float);


#endif /* STEERINGCONTROL_H_ */
