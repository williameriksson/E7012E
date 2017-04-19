#include "lineSensorReader.h"
//TODO: add code here.

//linesensor takes input on port PB0 - PB8.
LineSensorArray lsr;

void initLineSensorReader() {
	GPIOB->MODER &= ~(0x3FFFF); //sets MODER 0-8 to 00 (input mode) (sets PB0-PB8 for GPIO input mode)
}

void updateLineSensorArray() {
	int sensorArrayStatusBits = (GPIOB->IDR & 0x1FF); //applies bitmask for bit 0-8 of Input data register.
	for(int i = 0; i < LINESENSORARRAY_SIZE; i++) {
		lsr.values[i] = sensorArrayStatusBits | (int)pow((double)2, (double)i);
	}
}

float getDistanceOffset() {
	int maxSensorDistanceOffset = LINESENSORARRAY_SIZE / 2;
	int distanceFromCenter = 0;
	int activeSensorsCount = 0;
	for(int i = 0; i < LINESENSORARRAY_SIZE; i++) {
		if(lsr.values[i] == 1) {
			distanceFromCenter += maxSensorDistanceOffset - i;
			activeSensorsCount++;
		}
	}
	return (float)distanceFromCenter / (float)activeSensorsCount;
}



