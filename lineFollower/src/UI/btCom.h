#ifndef UI_BTCOM_H_
#define UI_BTCOM_H_

#include "stm32f4xx.h"
#include "stdlib.h"
#include "string.h"
#include "../Controller/PIDcontroller.h"
#include "../Controller/controlLoopHandler.h"

#define BAUDRATE 9600
#define RECIEVE_BUFFERSIZE 32
#define TUNE 1
#define CONTROL 2

#define ASCII_TM 0x544D
#define ASCII_TS 0x5453
#define ASCII_C 67
#define ASCII_F 70
#define DELIMITER 58 //ascii ":"
#define ENDSTRING 59 //ascii ";"

void initUART(void);
void sendData(float);
void runCommand(uint8_t*);
void tunePID(PID*, uint8_t*);

#endif /* UI_BTCOM_H_ */
