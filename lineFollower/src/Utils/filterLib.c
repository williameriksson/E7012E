#include "filterLib.h"

float continuesLPF(float prevSmoothData, float curRawData, float betaLPF) {
	return prevSmoothData - (betaLPF * (prevSmoothData - curRawData));
}
