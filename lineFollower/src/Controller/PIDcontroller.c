#include "PIDcontroller.h"

//initializes the controller with specified reference and PID params.
void initController(PID *controller, float ref, float Kp, float Ki, float Kd, int looptime) {
	controller->referencePoint = ref;
	controller->Kp = Kp;
	controller->Ki = Ki;
	controller->Kd = Kd;
	controller->integralError = 0.0f;
	controller->previousError = 0.0f;
	controller->looptime = looptime;
}

/*
 * Calculates the control-effort required based on current value.
 */
float runController(PID *controller, float currentValue) {
	float error = (controller->referencePoint - currentValue);
	float derivative = error - controller->previousError;
	float integral = controller->integralError + error;
	float output = controller->Kp * error + (controller->Ki * integral * controller->looptime) + (controller->Kd * derivative /controller->looptime);
	controller->previousError = error;
	return output;
}

void changeReference(PID *controller, float newRef) {
	controller->referencePoint = newRef;
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
}

