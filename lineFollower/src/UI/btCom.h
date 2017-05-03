#ifndef UI_BTCOM_H_
#define UI_BTCOM_H_

#include "stm32f4xx.h"
#include "stdlib.h"
#include "string.h"
#include "../Controller/PIDcontroller.h"
#include "../Controller/controlLoopHandler.h"
#include "../Utils/stringLib.h"

#define BAUDRATE 9600
#define RECIEVE_BUFFERSIZE 64
#define TUNE 1
#define CONTROL 2

#define ASCII_TM 0x544D //Tune Motor
#define ASCII_TS 0x5453 //Tune Servo
#define ASCII_F0 0x4630 //Fetch Params
#define ASCII_CV 0x4356 //Change ref-velocity of car
#define ASCII_GV 0x4756 //Get Velocity of car
#define DELIMITER 58 //ascii ":"
#define ENDSTRING 59 //ascii ";"

void initUART(void);
void sendData(float);
void runCommand(uint8_t*);
void tunePID(PID*, uint8_t*);
void pushPIDparams(PID*, PID*);
void changeVelocity(PID*, uint8_t*);

#endif /* UI_BTCOM_H_ */
