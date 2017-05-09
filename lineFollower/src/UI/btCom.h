#ifndef UI_BTCOM_H_
#define UI_BTCOM_H_

#include "stm32f4xx.h"
#include "stdlib.h"
#include "string.h"
#include "../Controller/PIDcontroller.h"
#include "../Controller/controlLoopHandler.h"
#include "../Utils/stringLib.h"

#define BAUDRATE 9600
#define RECIEVE_BUFFERSIZE 80

#define ASCII_TM 0x544D //Tune Motor controller
#define ASCII_TS 0x5453 //Tune Servo controller
#define ASCII_F0 0x4630 //Triggers sending of PID params.
#define ASCII_CV 0x4356 //Change ref-velocity of car
#define ASCII_GV 0x4756 //Get Velocity of car
#define ASCII_LM 0x4C4D //Sets the Lowpass beta for Motor controller
#define ASCII_LS 0x4C53 //Sets the Lowpass beta for Steering controller
#define ASCII_SV 0x5346 //Triggers sending of velocity (speed)
#define DELIMITER 58 //ascii ":"
#define ENDSTRING 59 //ascii ";"

void initUART(void);
void sendData(char*, int);
void runCommand(uint8_t*);
void tunePID(PID*, uint8_t*);
void tunePIDlowpassBeta(PID*, uint8_t*);
void pushPIDparams(PID*, PID*);
void changeVelocity(PID*, uint8_t*);
void sendVelocity();

#endif /* UI_BTCOM_H_ */
