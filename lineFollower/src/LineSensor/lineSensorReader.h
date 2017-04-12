/*
 * lineSensorReader.h
 *
 *  Created on: 11 Apr 2017
 *      Author: sebas
 */

#ifndef LINESENSORREADER_H_
#define LINESENSORREADER_H_
#define LINESENSORARRAY_SIZE 9

#include "stm32f4xx.h"

typedef struct {
	uint8_t values[LINESENSORARRAY_SIZE];
} LineSensors;

void initLineSensorReader(void);
void updateLineSensors(void);

#endif /* LINESENSORREADER_H_ */
