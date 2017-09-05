#include "btCom.h"


//CircularBUFFER recieveBuffer;
uint8_t indata[RECIEVE_BUFFERSIZE];
int indata_index;

void initUART () {
	__disable_irq();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 24);
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 28);
	USART6->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USART6->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
	USART6->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART6->CR1 |= USART_CR1_TE;
	USART6->BRR |= 50000000L/BAUDRATE;

	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 71);
	__enable_irq();

//	circularBufferInit(&recieveBuffer, (uint8_t)0, (uint8_t)RECIEVE_BUFFERSIZE);
//	fillBuffer(&recieveBuffer, 0);
	indata_index = 0;
}


void USART6_IRQHandler (void) {
	uint8_t ch;
	if(USART6->SR & USART_SR_RXNE) {
		ch = (uint8_t) USART6->DR;
		USART6->SR &= ~USART_SR_RXNE;
		if((int)ch != (int)END_OF_PACKET) {
			indata[indata_index] = ch;
			indata_index++;
		}
		else {
			indata[indata_index] = ch;
			uint8_t decoded[indata_index-1];
			unStuffData(indata, indata_index+1, decoded);
			runCommand(decoded);
			indata_index = 0;
		}
	}
}

void runCommand(uint8_t *commandString) {
	uint16_t command = (commandString[0] << 8) | (commandString[1]); //extracts the 2 first bytes of commandString (for instruction interpretation)
	switch(command) {
		case ASCII_TM: //Tuning PID motor params.
			tunePID(&motorPID, commandString);
			break;
		case ASCII_TS: //Tuning PID steering params.
			tunePID(&steeringPID, commandString);
			break;
		case ASCII_F0: //PC fetching parameters.
			pushPIDparams(&motorPID, &steeringPID);
			break;
		case ASCII_CV: //Change velocity
			changeVelocity(&motorPID, commandString);
			break;
		case ASCII_LM: //Change lowpass beta for motor controller.
			tunePIDlowpassBeta(&motorPID, commandString);
			break;
		case ASCII_LS: //Change lowpass beta for steering controller.
			tunePIDlowpassBeta(&steeringPID, commandString);
			break;
		case ASCII_SV: //Send velocity
			sendVelocity();
			break;
		case ASCII_EM: //error motor
			sendPIDerror(&motorPID);
			break;
		case ASCII_ES: //error steering
			sendPIDerror(&steeringPID);
			break;
		case ASCII_OM: //output motor
			sendPIDoutput(&motorPID);
			break;
		case ASCII_OS: //output steering
			sendPIDoutput(&steeringPID);
			break;
		case ASCII_EO: //error & output all
			sendPIDErrOut(&steeringPID, &motorPID);
			break;
		case ASCII_OD: //obstaclesensor distance tuning
			changeObstacleDistanceThreshhold(commandString);
			break;
		case ASCII_OT: //obstaclesensor time tuning
			changeObstacleAvoidanceTime(commandString);
			break;
		case ASCII_FF: //sets feedforward multiplier
			tuneFeedForward(commandString);
			break;
		case ASCII_S0:
			stopControllers();
			break;
		case ASCII_S1:
			startControllers();
			break;
		case ASCII_TO:
			setObstacleDetection(commandString);
			break;
	}
}

//tunePID recieves parameters on the format PID (concatenated floats (12 bytes total))
void tunePID(PID *controller, uint8_t *cmd) {
	int index = 2;
	float pidValues[3];
	byteArrayToFloat(cmd+index, 12, pidValues);
	changeParameters(controller, pidValues[0], pidValues[1], pidValues[2]);
}

void tunePIDlowpassBeta(PID *controller, uint8_t *cmd) {
	int index = 2;
	float lowpassBeta;
	byteArrayToFloat(cmd+index, 4, &lowpassBeta);
	changeLowpassBeta(controller, lowpassBeta);
}

void changeVelocity(PID *motorC, uint8_t *cmd) {
	int index = 2;
	float newVelocity;
	byteArrayToFloat(cmd+index, 4, &newVelocity);
	changeReference(motorC, newVelocity);
}

void changeObstacleDistanceThreshhold(uint8_t *cmd) {
	int index = 2;
	float newThreshhold;
	byteArrayToFloat(cmd+index, 4, &newThreshhold);
	setObstacleThreshhold(newThreshhold);
}

void changeObstacleAvoidanceTime(uint8_t *cmd) {
	int index = 2;
	float newTime;
	byteArrayToFloat(cmd+index, 4, &newTime);
	setObstacleAvoidanceTimer(newTime);
}

void setObstacleDetection(uint8_t *cmd) {
	int index = 2;
	float status;
	byteArrayToFloat(cmd+index, 4, &status);
	int temp = (int)status;
	toggleObstacleSensor(temp);
}

void tuneFeedForward(uint8_t *cmd) {
	int index = 2;
	float newConstant;
	byteArrayToFloat(cmd+index, 4, &newConstant);
	setFeedforwardConstant(newConstant);
}

void sendVelocity() {
	uint8_t unencoded[4];
	uint8_t encoded[6];
	floatToByteArray(&speed, 4, unencoded);
	stuffData(unencoded, 4, encoded);
	sendData(encoded, 6);
}

void sendPIDerror(PID *controller) {
	uint8_t unencoded[4];
	uint8_t encoded[6];
	floatToByteArray(&controller->previousError, 4, unencoded);
	stuffData(unencoded, 4, encoded);
	sendData(encoded, 6);
}

void sendPIDoutput(PID *controller) {
	uint8_t unencoded[4];
	uint8_t encoded[6];
	floatToByteArray(&controller->previousOutput, 4, unencoded);
	stuffData(unencoded, 4, encoded);
	sendData(encoded, 6);
}

void sendPIDErrOut(PID *steering, PID *motor) {
	uint8_t unencoded[8];
	uint8_t encoded[10];
	float data[2];
	data[0] = steering->previousError;
	data[1] = totalAdjustment;
	floatToByteArray(data, 2, unencoded);
	stuffData(unencoded, 8, encoded);
	sendData(encoded, 10);
}

void pushPIDparams(PID *motorC, PID *steeringC) {
	int dataSize = 12;
	float rawData[dataSize];
	uint8_t unencoded[dataSize*4];
	uint8_t encoded[(dataSize*4)+2];

	rawData[0] = motorC->referencePoint;
	rawData[1] = motorC->Kp;
	rawData[2] = motorC->Ki;
	rawData[3] = motorC->Kd;
	rawData[4] = motorC->betaLPF;
	rawData[5] = steeringC->Kp;
	rawData[6] = steeringC->Ki;
	rawData[7] = steeringC->Kd;
	rawData[8] = steeringC->betaLPF;
	rawData[9] = getFeedforwardConstant();
	rawData[10] = getObstacleThreshhold();
	rawData[11] = getObstacleAvoidanceTimer();

	floatToByteArray(rawData, dataSize, unencoded);
	stuffData(unencoded, dataSize*4, encoded);
	sendData(encoded, (dataSize*4)+2);
}

void sendData(uint8_t *sendData, int size) {
	int i = 0;
	while((i < size)) {
		USART6->DR = sendData[i];
		i++;
		while(!(USART6->SR & USART_SR_TXE)) {
			//wait for transmission
		}
	}
}
