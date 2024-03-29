#ifndef CONTROLLER_PIDCONTROLLER_H_
#define CONTROLLER_PIDCONTROLLER_H_

#include "Utils/filterLib.h"

typedef struct {
	float referencePoint; //reference point to control around
	float Kp, Ki, Kd; //PID parameters
	float integralError; //the accumulated integral error
	float previousError; //the calculated error in the previous loop
	int looptime; //time (in ms) between controller runs
	float prevFilteredSignal;
	float betaLPF; //beta for LPF filter
	float previousOutput;
}PID;

void initController(PID*, float, float, float, float, int, float);
float runController(PID*, float, int);
void changeReference(PID*, float);
void changeLowpassBeta(PID*, float);
void changeParameters(PID*, float, float, float);
void resetPIDError(PID*);

#endif /* CONTROLLER_PIDCONTROLLER_H_ */
