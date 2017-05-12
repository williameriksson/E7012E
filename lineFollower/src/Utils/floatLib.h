
#ifndef UTILS_FLOATLIB_H_
#define UTILS_FLOATLIB_H_

#include <stdint.h>

typedef union {
	float f;
	uint8_t byte[4];
} floatByte;

void floatToByteArray(float*, int, uint8_t*);
void byteArrayToFloat(uint8_t*, int, float*);

#endif /* UTILS_FLOATLIB_H_ */
