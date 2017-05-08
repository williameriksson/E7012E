#include "main.h"

int main(void)
{

	initSteeringControl();
	initHallSensor();
//	initUserButton();
	initMotorControl();
    initLineSensorReader();
    initControlLoopHandler();
    initUART();

	while (1) {

	}
}
