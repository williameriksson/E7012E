#include <HallSensor/hallSensorReader.h>



// Converts time diff (uS) between 2 magnets of the 8 magnet setup to meters per second
//#define usToMpsFourM(t) ((2.0 * 3.141592 * 0.04) / 4) * 100000 / t
#define usToMpsEightM(t) 3.25 * 2.0 * 3.141592 * 100.0 / ((float)t * 8.0)

CircularBUFFER hallBuffer;
CircularBUFFER directionBuffer;

void initHallSensor() {
	__disable_irq();
	circularBufferInit(&hallBuffer, 0, 4);

	speed = 0;
	fillBuffer(&directionBuffer, 1);
	fillBuffer(&hallBuffer, 65000);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock

//	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable clock for TIM5
//	TIM5->ARR = 0xFFFFFFFF; // Auto reload at max
//	TIM5->PSC = 10000 - 1; // Prescale to 10kHz
//	//TIM5->DIER |= TIM_DIER_UIE;
//	TIM5->CR1 |= TIM_CR1_CEN; // Enable TIM5

	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; // Enable clock for TIM11
	TIM11->ARR = 0xFFFF; // Auto reload
	TIM11->PSC = 10000 - 1; // Prescaler
	TIM11->CCMR1 |= TIM_CCMR1_CC1S_0; // Configure channel CC1 as input, IC1 is mapped on TI1
	// CCMR1 offers different filtering settings in the IC1F field, might wanna look that up.
	// TIM11->CCMR1 |= TIM_CCMR1_IC1F_1;
	TIM11->CCER |= TIM_CCER_CC1P; // Capture on falling edge
	TIM11->CCER |= TIM_CCER_CC1E; // Enable capture
	TIM11->DIER |= TIM_DIER_CC1IE; // Enable capture interrupt
	//TIM11->DIER |= TIM_DIER_UIE;
	TIM11->CR1 |= TIM_CR1_CEN; // Enable TIM11
	// Read captured value from TIM11->CCR1

	GPIOB->MODER |= GPIO_MODER_MODER9_1; // Set PB9 to AF mode
	GPIOB->AFR[1] |= (((uint8_t)0x03) << 4); // Select TIM11 CH1 as AF for PB9


//	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB; // Set external interrupt EXTI4 for PB4
//	EXTI->FTSR |= EXTI_FTSR_TR4; // Enable interrupt on falling edge for TR4
//	EXTI->RTSR |= EXTI_FTSR_TR4; // Enable interrupt on falling edge for TR4
//	EXTI->IMR |= EXTI_IMR_MR4; // Unmask the interrupt register for MR4 (Active for PB4)

//For Direction
//	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB; // Set external interrupt EXTI4 for PB5
//	EXTI->RTSR |= EXTI_RTSR_TR5; // Enable interrupt on rising edge for TR5
//	EXTI->FTSR |= EXTI_FTSR_TR5; // Enable interrupt on falling edge for TR5
//	EXTI->IMR |= EXTI_IMR_MR5; // Unmask the interrupt register for MR5 (Active for PB5)


//	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
//	TIM10->ARR = 1000; // Auto reload
//	TIM10->PSC = 10000 - 1; // Prescale to 10kHz
//	//TIM10->DIER |= TIM_DIER_UIE;
//	TIM10->CR1 |= TIM_CR1_CEN; // Enable TIM5


	__enable_irq(); //Enable global interrupts


	//NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 15); // Set the priority, this should probably be changed..
	//NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn ); // Enable the interrupt

//
//	NVIC_SetPriority(EXTI4_IRQn, 15); // Set the priority, this should probably be changed..
//	NVIC_EnableIRQ(EXTI4_IRQn); // Enable the interrupt

	NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 15);
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}


void TIM1_TRG_COM_TIM11_IRQHandler () {
	static uint16_t startTime = 0;
	if (TIM11->SR & TIM_SR_CC1IF) { // Check capture interrupt flag
		uint16_t endTime = TIM11->CCR1;
		uint16_t diff = endTime - startTime;
		pushBuffer(&hallBuffer, diff);
		uint16_t filteredValue = getBufferAverage(&hallBuffer);
		//speed = (2.0f * 10000.0f) / (2.0f * (float)filteredValue);
		speed = usToMpsEightM(filteredValue);
		startTime = endTime;
		TIM11->SR &= ~(TIM_SR_CC1IF); // Clear capture flag
	}
//	else if (TIM11->SR & TIM_SR_UIF) {
//		speed = 0.0;
//		TIM11->SR &= ~(TIM_SR_UIF);
//	}


}
