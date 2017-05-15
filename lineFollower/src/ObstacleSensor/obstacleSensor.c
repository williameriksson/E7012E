#include "obstacleSensor.h"

CircularBUFFER obstacleBuffer;
void initObstacleSensor() {
	__disable_irq();
	obstacleDistance = 250.0f;
	obstacleThreshold = 70.0f;
	circularBufferInit(&obstacleBuffer, 0, 5);
	fillBuffer(&obstacleBuffer, 15000);


	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; // Enable clock for TIM10
	TIM10->PSC = 10000-1; // Prescale to 10kHz
	TIM10->ARR = 0xFFFF; // Auto reload at max
	TIM10->CR1 |= TIM_CR1_CEN; // Enable TIM11

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock port C
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; // Set external interrupt EXTI4 for PC4

	EXTI->RTSR |= EXTI_RTSR_TR4; // Enable interrupt on rising edge for TR4
	EXTI->FTSR |= EXTI_FTSR_TR4; // Enable interrupt on falling edge for TR4
	EXTI->IMR |= EXTI_IMR_MR4; // Unmask the interrupt register for MR4 (Active for PC4)

	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	__enable_irq();
	NVIC_SetPriority(EXTI4_IRQn, 23); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI4_IRQn); // Enable the interrupt
}

float getDistance() {
 return 0.0;
}

float getObstacleThreshhold() {
	return obstacleThreshold;
}

void setObstacleThreshhold(float newValue) {
	obstacleThreshold = newValue;
}

void EXTI4_IRQHandler (void) {
	static int startTime = 0;

	if (EXTI->PR & EXTI_PR_PR4) {	// Check interrupt flag for PR4
		if (GPIOC->IDR & 16) {		// Check if rising edge
			startTime = TIM10->CNT;
		} else {
			int endTime = TIM10->CNT;
			int duration = endTime - startTime;
			if (duration < 0) {
				duration = (0xFFFF - startTime) + endTime;
			}
			pushBuffer(&obstacleBuffer, duration);
			int averageDuration = getBufferAverage(&obstacleBuffer);
			obstacleDistance = ((float)averageDuration / 1.47f) * 2.54f; // 147 uS per cm
			if (obstacleDistance <= obstacleThreshold) {
				GPIOA->ODR |= (1 << 8);
			} else {
				GPIOA->ODR &= ~(1 << 8);
			}
		}
	}

	EXTI->PR |= EXTI_PR_PR4; // clear interrupt flag PR4 by writing 1
}






//float obstacleDistance;
//void initObstacleSensor() {
//	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 	// Enable perip. clock for ADC1
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	// Enable clock for GPIOC
//
//	GPIOC->MODER |= GPIO_MODER_MODER4; 		// Set PC4 to Analog mode
//
//	ADC->CCR |= ADC_CCR_ADCPRE;				// Prescale ADC clock by 8 -> 12.5Mhz
//	ADC1->SMPR1 |= ADC_SMPR1_SMP14; 		// Set sample rate for channel 14 to (111 = 480 cycles)
//	ADC1->CR2 |= ADC_CR2_CONT;				// Set to continuous mode
//	//ADC1->CR1 |= ADC_CR1_EOCIE;			// Enables interrupts for EOC
//	//ADC1->CR2 |= ADC_CR2_EOCS;
//
//	ADC1->SQR3 |= 14;						// PC4 is on channel 14
//	//ADC1->SQR1 |= ADC_SQR1_L_0
//	ADC1->CR2 |= ADC_CR2_ADON; 				// Enable ADC1
//	ADC1->CR2 |= ADC_CR2_SWSTART;			// Start the conversions
//
//	// NVIC_EnableIRQ(ADC_IRQn);			//Enables interrupt handler
//	// NVIC_SetPriority(ADC_IRQn, 18);
//}
//
//float getDistance() {
//	float voltage = ADC1->DR / 1241.0f;
//	obstacleDistance = (2.37692 - voltage) / 0.00384615;
//	return obstacleDistance;
//
//}

//void ADC_IRQHandler (void) {
//	if(ADC1->SR & ADC_SR_EOC) {
//		float distance = getDistance();
//
//		ADC1->SR &= ~ADC_SR_EOC;
//	}
//
//}
