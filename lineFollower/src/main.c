#include "main.h"

/*
 * temporary function to satisfy requirements in LAB2
 * TODO: REMOVE THIS
 */
void lab2blinky() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enables clock on GPIOB
	GPIOB->MODER &= ~GPIO_MODER_MODER3; //clears MODER3 reg.
	GPIOB->MODER |= GPIO_MODER_MODER3_0; //sets GPIOB mode to general output on pin PB3

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enables TIM2 timer peripherals
	TIM3->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM3->PSC = 100-1; //sets prescalar -> clock freq 1MHz
	TIM3->ARR = 100000-1; //10Hz freq (cycle time)
	TIM3->DIER |= TIM_DIER_UIE; //enables interrupt on ARR overflow
	TIM3->CR1 |= TIM_CR1_CEN; //enables the timer.
	__enable_irq();

	NVIC_SetPriority(TIM3_IRQn, 50); //sets priority
	NVIC_EnableIRQ(TIM3_IRQn); //enables the interrupt


}

void TIM3_IRQHandler(void) {
	if(TIM3->SR && TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF; //Resets the interrupt flag
		GPIOB->ODR ^= GPIO_ODR_ODR_3; //toggles PB3;
	}
}

int main(void)
{
  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */
//	initServoControl();
	initUserButton();
	initMotorControl();
//	lab2blinky();

	while (1) {

	}
}
