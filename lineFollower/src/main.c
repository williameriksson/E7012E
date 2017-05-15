#include "main.h"

int main(void)
{

	initSteeringControl();
	initHallSensor();
//	initUserButton();
	initMotorControl();
    initLineSensorReader();
    initControlLoopHandler();
    initObstacleSensor();
    initUART();

	while (1) {

	}
}
