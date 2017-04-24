#ifndef CONTROLLER_PIDCONTROLLER_H_
#define CONTROLLER_PIDCONTROLLER_H_

#include "../UI/btCom.h"

typedef struct {
	float referencePoint; //reference point to control around
	float Kp, Ki, Kd; //PID parameters
	float integralError; //the accumulated integral error
	float previousError; //the calculated error in the previous loop
	int looptime; //time (in ms) between controller runs
}PID;

void initController(PID*, float, float, float, float, int);
float runController(PID*, float);
void changeReference(PID*, float);
void resetPIDError(PID*);

#endif /* CONTROLLER_PIDCONTROLLER_H_ */
