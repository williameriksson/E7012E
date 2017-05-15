
#ifndef CONTROLLER_FEEDFORWARDCONTROLLER_H_
#define CONTROLLER_FEEDFORWARDCONTROLLER_H_

typedef struct {
	float referencePoint; //reference point to control around
	float Kp, Ki, Kd; //PID parameters
	float integralError; //the accumulated integral error
	float previousError; //the calculated error in the previous loop
	int looptime; //time (in ms) between controller runs
	float prevFilteredSignal;
	float betaLPF; //beta for LPF filter
	float previousOutput;
}FFController;

#endif /* CONTROLLER_FEEDFORWARDCONTROLLER_H_ */
