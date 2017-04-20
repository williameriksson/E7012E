#ifndef LINESENSORREADER_H_
#define LINESENSORREADER_H_
#define LINESENSORARRAY_SIZE 9

#include "stm32f4xx.h"
#include "math.h"

typedef struct {
	uint8_t values[LINESENSORARRAY_SIZE];
} LineSensorArray;

void initLineSensorReader(void);
void updateLineSensorArray();
float updateDistanceOffset();

#endif /* LINESENSORREADER_H_ */
