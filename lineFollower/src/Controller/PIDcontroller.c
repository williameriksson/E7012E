#include "PIDcontroller.h"

//initializes the controller with specified reference and PID params.
void initController(PID *controller, float ref, float Kp, float Ki, float Kd, int looptime, float lpf) {
	controller->referencePoint = ref;
	controller->Kp = Kp;
	controller->Ki = Ki;
	controller->Kd = Kd;
	controller->integralError = 0.0f;
	controller->previousError = 0.0f;
	controller->looptime = looptime;
	controller->prevFilteredSignal = 0.0f;
	controller->betaLPF = lpf;
}

/*
 * Calculates the control-effort required based on current value.
 */
float runController(PID *controller, float currentValue, int isSaturated) {
	float error = (controller->referencePoint - currentValue);
	float filteredSignal = continuesLPF(controller->prevFilteredSignal, currentValue, controller->betaLPF);
	if(!isSaturated) {
		controller->integralError = controller->integralError + error;
	}
	controller->previousError = error;
	float derivative = controller->prevFilteredSignal - filteredSignal;
	controller->prevFilteredSignal = filteredSignal;
	float integral = controller->integralError;
	float output = controller->Kp * error + (controller->Ki * integral * ((float)controller->looptime / 1000.0f)) + (controller->Kd * derivative / ((float)controller->looptime / 1000.0f));
	return output;
}

void changeReference(PID *controller, float newRef) {
	controller->referencePoint = newRef;
}

void changeLowpassBeta(PID *controller, float newBeta) {
	controller->betaLPF = newBeta;
}

void changeParameters(PID *controller, float P, float I, float D) {
	controller->Kp = P;
	controller->Ki = I;
	controller->Kd = D;
	resetPIDError(controller);
}

void resetPIDError(PID *controller) {
	controller->integralError = 0.0f;
	controller->previousError = 0.0f;
	controller->prevFilteredSignal = 0.0f;
}

