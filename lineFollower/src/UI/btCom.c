#include "btCom.h"

#include "stdlib.h"
//#include "stdbool.h"
#include "string.h"


#define BAUDRATE 9600
#define USARTBUFFSIZE 16
#define TUNE 1
#define CONTROL 2


void initUART () {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 24);
	GPIOC->AFR[0] |= (((uint8_t)0x08) << 28);
	USART6->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	//USART6->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
	//USART6->CR1 &= ~USART_CR1_TXEIE; //Disable UART TXE Interrupt
	USART6->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART6->CR1 |= USART_CR1_TE;
	//USART6->CR1 |= USART_CR1_RE;
	USART6->BRR |= 50000000L/BAUDRATE;

//	NVIC_EnableIRQ(USART6_IRQn);
//	NVIC_SetPriority(USART6_IRQn, 71);
	//Usart2Put(8);
	//DMA_SxCR_EN;
}

//void initDMAReq() {
//	DMA2->HISR |= DMA_HISR_FEIF4;
//	DMA2->S2CR |= 0;
//
//}


char number[5];
int numPoint = 0;
float pidParams[3];
int pidPoint = 0;

int commandMode = 0;

void sendData(float data){
	if (USART6->SR & USART_SR_TXE) {
		USART6->DR = data;
	}
}

void USART6_IRQHandler (void) {
	if (USART6->SR & USART_SR_RXNE) {

	}

}

//void USART6_IRQHandler (void) {
//	char delimiter = ":"; //ascii 58
//	char endstring = ";"; //ascii 59
//	uint8_t ch;
//	if (USART6->SR & USART_SR_RXNE){
////		ch=(uint8_t)USART6->DR;
////		if((int)ch == 84) { //ascii "T"
////			commandMode = TUNE;
////		}
////		else if((int)ch == 67) { //ascii "C"
////			commandMode = CONTROL;
////		}
////		else {
////			switch(commandMode) {
////				case TUNE:
////					tunePID(ch);
////					break;
////				case CONTROL:
////					controlCar(ch);
////					break;
////				default:
////					break;
////			}
////		}
////	}
//	if (USART6->SR & USART_SR_TXE) {
//	}
//
//	USART6->CR1 &= ~USART_CR1_TXEIE;
//
//}

//void tunePID(uint8_t ch) {
//	if((int)ch == 59) { // ";" recieved
//		pidPoint = 0;
//		numPoint = 0;
//		Kp = pidParams[0];
//		Ki = pidParams[1];
//		Kd = pidParams[2];
//		USART6->DR = 84; //T
//		resetPID();
//		commandMode = 0;
//	}
//
//	else if((int)ch == 58) { // ":" recieved
//		float pidValue = (float)atof(number);
//
//		pidParams[pidPoint] = pidValue;
//		pidPoint++;
//		numPoint = 0;
//	}
//	else {
//		number[numPoint] = ch;
//		numPoint++;
//	}
//
//}
//
//int command = 0;
//
//void controlCar(uint8_t ch) {
//	USART6->DR = 67; //C
//	if((int)ch == 59) { // ";" recieved
//		commandMode = 0;
//		if(command == 1) {
//			startController();
//		}
//		else if(command == 0) {
//			stopController();
//			resetSpeed();
//		}
//		//command finished
//	}
//	else if((int)ch == 48){ //0 recieved
//		command = 0;
//	}
//	else if((int)ch == 49){ //1 recieved
//		command = 1;
//	}
//}
