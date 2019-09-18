#include "interrupts.h"

void EXTI0_IRQHandler(){
	EXTI->PR |= EXTI_PR_PR0; // Clear pending bit for EXTI0 line
	// Set clockwise rotation direction
	TIM3->CCER |= TIM_CCER_CC1P; // Set high polarity
	TIM3->CCER &= ~TIM_CCER_CC2P; // Set low polarity
	TIM15->CCER &= ~TIM_CCER_CC1P; // Set low polarity
	TIM15->CCER |= TIM_CCER_CC2P; // Set high polarity
	//TIM 3 out enable
	TIM3->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable
	TIM3->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable
	//TIM 15 out enable
	TIM15->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable
	TIM15->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable
	// Run timers to control stepper motor
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM15->CR1 |= TIM_CR1_CEN;
}

void EXTI1_IRQHandler() {
	EXTI->PR |= EXTI_PR_PR1; // Clear pending bit for EXTI3 line
	// Set counter-clockwise rotation direction
	TIM3->CCER &= ~TIM_CCER_CC1P; // Set low polarity
	TIM3->CCER &= ~TIM_CCER_CC2P; // Set low polarity
	TIM15->CCER |= TIM_CCER_CC1P; // Set high polarity
	TIM15->CCER |= TIM_CCER_CC2P; // Set high polarity
	//TIM 3 out enable
	TIM3->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable
	TIM3->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable
	//TIM 15 out enable
	TIM15->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable
	TIM15->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable
	// Run timers to control stepper motor
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM15->CR1 |= TIM_CR1_CEN;
}

void EXTI4_IRQHandler() {
	EXTI->PR |= EXTI_PR_PR4; // Clear pending bit for EXTI4 line
	//TIM 3 out disable
	TIM3->CCER &= ~TIM_CCER_CC1E; // Output state channel 1 enable
	TIM3->CCER &= ~TIM_CCER_CC2E; // Output state channel 2 enable
	//TIM 15 out disable
	TIM15->CCER &= ~TIM_CCER_CC1E; // Output state channel 1 enable
	TIM15->CCER &= ~TIM_CCER_CC2E; // Output state channel 2 enable
	// Run timers to control stepper motor
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM15->CR1 &= ~TIM_CR1_CEN;
}

void ADC1_IRQHandler() {
	/* Clear the selected ADC interrupt pending bits */
	ADC1->SR = 0;
	NVIC_SystemReset();
}
