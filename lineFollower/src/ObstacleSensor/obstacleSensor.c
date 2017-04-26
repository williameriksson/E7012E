#include "obstacleSensor.h"

void initObstacleSensor() {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 	// Enable perip. clock for ADC1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	// Enable clock for GPIOC

	GPIOC->MODER |= GPIO_MODER_MODER4; 		// Set PC4 to Analog mode

	ADC->CCR |= ADC_CCR_ADCPRE;				// Prescale ADC clock by 8 -> 12.5Mhz
	ADC1->SMPR1 |= ADC_SMPR1_SMP14; 		// Set sample rate for channel 14 to (111 = 480 cycles)
	//ADC1->CR2 |= ADC_CR2_CONT;				// Set to continous mode
	ADC1->CR1 |= ADC_CR1_EOCIE;			//Enables interrupts for EOC
	ADC1->CR2 |= ADC_CR2_ADON; 				// Enable ADC1
	//ADC1->CR2 |= ADC_CR2_SWSTART;			// Start the conversions
	ADC1->SQR3 |= 14;

	NVIC_EnableIRQ(ADC_IRQn);			//Enables interrupt handler
	NVIC_SetPriority(ADC_IRQn, 18);
}

float getDistance() {
	float voltage = ADC1->DR / 1241.0f;
	float obstacleDistance = (2.975 - voltage) / 0.02375;
	return obstacleDistance;
	//return obstacleDistance;
}

void ADC_IRQHandler (void) {
	if(ADC1->SR & ADC_SR_EOC) {
		float distance = getDistance();

		ADC1->SR &= ~ADC_SR_EOC;
	}

}
