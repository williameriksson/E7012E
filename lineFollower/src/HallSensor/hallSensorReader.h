

#ifndef HALLSENSOR_HALLSENSORREADER_H_
#define HALLSENSOR_HALLSENSORREADER_H_
#include "stm32f4xx.h"
#include <Utils/circularBuffer.h>

void initHallSensor(void);

float speed;
int magnetTick;
float traveledDistance;
#endif /* HALLSENSOR_HALLSENSORREADER_H_ */
