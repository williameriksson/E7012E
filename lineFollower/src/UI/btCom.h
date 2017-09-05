#ifndef UI_BTCOM_H_
#define UI_BTCOM_H_

#include <stm32f4xx.h>
#include "stdlib.h"
#include <string.h>
#include "../Controller/PIDcontroller.h"
#include "../Controller/controlLoopHandler.h"
#include "../Utils/stringLib.h"
#include "../Utils/cobs.h"
#include "../Utils/floatLib.h"
#include "../ObstacleSensor/obstacleSensor.h"

#define BAUDRATE 9600
#define RECIEVE_BUFFERSIZE 80

#define ASCII_S0 0x5330 //stop controllers
#define ASCII_S1 0x5331 //start controllers
#define ASCII_TO 0x544F //toggle obstaclesensor
#define ASCII_TM 0x544D //Tune Motor controller
#define ASCII_TS 0x5453 //Tune Servo controller
#define ASCII_F0 0x4630 //Triggers sending of PID params.
#define ASCII_CV 0x4356 //Change ref-velocity of car
#define ASCII_GV 0x4756 //Get Velocity of car
#define ASCII_LM 0x4C4D //Sets the Lowpass beta for Motor controller
#define ASCII_LS 0x4C53 //Sets the Lowpass beta for Steering controller
#define ASCII_OD 0x4F44 //Sets the obstaclesensor distance for avoidance maneuver
#define ASCII_OT 0x4F54 //Sets the obstaclesensor avoidance time
#define ASCII_FF 0x4646 //Sets the feed-forward multiplier
#define ASCII_SV 0x5346 //Triggers sending of velocity (speed)
#define ASCII_SS 0x5353 //Triggers sending of steering
#define ASCII_ES 0x4553	//Triggers sending of PID error steering
#define ASCII_EM 0x454D //Triggers sending of PID error motor
#define ASCII_OS 0x4F53 //Triggers sending of PID output steering
#define ASCII_OM 0x4F4D //Triggers sending of PID output motor
#define ASCII_EO 0x454F //Triggers sending of PID error (motor & steering) and PID output (motor & steering)

#define END_OF_PACKET 0x00

void initUART(void);
void sendData(uint8_t*, int);
void runCommand(uint8_t*);
void tunePID(PID*, uint8_t*);
void tunePIDlowpassBeta(PID*, uint8_t*);
void pushPIDparams(PID*, PID*);
void changeVelocity(PID*, uint8_t*);
void changeObstacleDistanceThreshhold(uint8_t*);
void changeObstacleAvoidanceTime(uint8_t*);
void setObstacleDetection(uint8_t*);
void tuneFeedForward(uint8_t*);
void sendVelocity();
void sendPIDerror(PID*);
void sendPIDoutput(PID*);
void sendPIDErrOut(PID*, PID*);

#endif /* UI_BTCOM_H_ */
