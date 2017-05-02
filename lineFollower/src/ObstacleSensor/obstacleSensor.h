#ifndef OBSTACLESENSOR_OBSTACLESENSOR_H_
#define OBSTACLESENSOR_OBSTACLESENSOR_H_

#include "stm32f4xx.h"
#include "../Utils/circularBuffer.h"

float obstacleDistance;
float obstacleThreshold;
void initObstacleSensor(void);
float getDistance(void);


#endif /* OBSTACLESENSOR_OBSTACLESENSOR_H_ */
