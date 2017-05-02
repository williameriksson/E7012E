#include "btCom.h"
#include <string.h>

CircularBUFFER recieveBuffer;


void initUART () {
	__disable_irq();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 24);
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 28);
	USART6->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USART6->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
	//USART6->CR1 &= ~USART_CR1_TXEIE; //Disable UART TXE Interrupt
	USART6->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART6->CR1 |= USART_CR1_TE;
	//USART6->CR1 |= USART_CR1_RE;
	USART6->BRR |= 50000000L/BAUDRATE;

	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 71);
	//Usart2Put(8);
	//DMA_SxCR_EN;
	__enable_irq();

	circularBufferInit(&recieveBuffer, (uint8_t)0, (uint8_t)RECIEVE_BUFFERSIZE);
	fillBuffer(&recieveBuffer, 0);
}

void sendData(float data){
	if (USART6->SR & USART_SR_TXE) {
		USART6->DR = data;
	}
}
//TM:1.0:1.0:1.0:;
void USART6_IRQHandler (void) {
	uint8_t ch;
	if(USART6->SR & USART_SR_RXNE) {
		ch = (uint8_t) USART6->DR;
		if((int)ch != ENDSTRING) {
			pushBuffer(&recieveBuffer, ch);
		}
		else {
			pushBuffer(&recieveBuffer, ch);
			uint8_t commandString[recieveBuffer.indexPointer+1];
			for(int i = 0; i < recieveBuffer.indexPointer; i++) {
				commandString[i] = pullBuffer(&recieveBuffer, i-recieveBuffer.indexPointer);
			}
			runCommand(commandString);
			resetBuffer(&recieveBuffer);
		}
	}
}

void runCommand(uint8_t *commandString) {
	uint16_t command = (commandString[0] << 8) | (commandString[1]); //extracts the 2 first chars of commandString (for instruction interpretation)
//	int i = 0;
//	while(i < recieveBuffer.indexPointer) {
//		char kek = commandString[i];
//		char kek = pullBuffer(&recieveBuffer, i-recieveBuffer.indexPointer);
//		i++;
//	}
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
		case ASCII_CV:
			break;
	}
}

void tunePID(PID *controller, uint8_t *cmd) {
	int index = 3;
	float pidValues[3];
	int pidPointer = 0;
	while((int)cmd[index] != (int)ENDSTRING) {
//	while(index < sizeof(*cmd)) {
		char number[5];
		int numPointer = 0;
		while((int)cmd[index] != (int)DELIMITER) {
			number[numPointer] = cmd[index];
			index++;
			numPointer++;
		}
		pidValues[pidPointer] = (float)atof(number);
		index++;
		pidPointer++;
	}
	changeParameters(controller, pidValues[0], pidValues[1], pidValues[2]);
}

void pushPIDparams(PID *motorC, PID *steeringC) {
	int numSize = 7;
	char sendString[49];
	ftoa(motorC->referencePoint, sendString, 4);
	sendString[numSize * 1 - 1] = (char)DELIMITER;
	int indexer = (numSize * 1);
	ftoa(motorC->Kp, sendString+indexer, 4);
	sendString[numSize * 2 - 1] = (char)DELIMITER;
	indexer += numSize;
	ftoa(motorC->Ki, sendString+indexer, 4);
	sendString[numSize * 3 - 1] = (char)DELIMITER;
	indexer += numSize;
	ftoa(motorC->Kd, sendString+indexer, 4);

	sendString[numSize * 4 - 1] = (char)DELIMITER;
	indexer += numSize;
	ftoa(steeringC->Kp, sendString+indexer, 4);
	sendString[numSize * 5 - 1] = (char)DELIMITER;
	indexer += numSize;
	ftoa(steeringC->Ki, sendString+indexer, 4);
	sendString[numSize * 6 - 1] = (char)DELIMITER;
	indexer += numSize;
	ftoa(steeringC->Kd, sendString+indexer, 4);

	sendString[(numSize * 7) - 1] = (char)ENDSTRING;

	int i = 0;
	while((i < 49)) {
		USART6->DR = sendString[i];
		i++;
		while(!(USART6->SR & USART_SR_TXE)) {
			//wait for transmission
		}
	}
}
