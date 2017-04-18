#include "lineSensorReader.h"
//TODO: add code here.

//linesensor takes input on port PB0 - PB8.
void initLineSensorReader() {
	GPIOB->MODER &= ~(0x3FFFF); //sets MODER 0-8 to 00 (input mode)
}

void updateLineSensors() {

}

