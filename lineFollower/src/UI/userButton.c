#include "userButton.h"

void initUserButton() {
	__disable_irq(); //disables global interrupts during configuration.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //enables the clock on GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enables
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->FTSR |= EXTI_FTSR_TR13; //sets interrupt on falling edge
	EXTI->IMR |= EXTI_IMR_MR13; //sets mask to allow interrupts MR13
	__enable_irq();

	NVIC_EnableIRQ(EXTI15_10_IRQn); //enables the interrupt
	NVIC_SetPriority(EXTI15_10_IRQn, 40);
}

//Variables for testing
int toggle = 0;
float tempSteer = -30.0f;
float tempSpeed = 0.0f;
int testPW = 15000;
int Count = 0;
//End Testing

void EXTI15_10_IRQHandler(void) {
	EXTI->PR |= EXTI_PR_PR13;

	if(toggle) {
		setSteering(15.0);
		toggle = 0;
	}
	else {
		setSteering(-15.0);
		toggle = 1;
	}

//	tempSpeed += 0.5f;
//	setSpeed(tempSpeed);
}
