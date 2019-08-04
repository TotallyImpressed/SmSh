#include "init.h"
#include "interrupts.h"

int main(void) {
	/* General initialization*/
	initComRCC();
	initGPIO();
	initTIM3();
	initTIM15();
	initIRPT();
	__enable_irq();

	while(1) {
//		if((GPIOA->IDR & GPIO_IDR_IDR0) == GPIO_IDR_IDR0) {
//			TIM3->CCER ^= TIM_CCER_CC1P;
////			TIM3->CCER ^= TIM_CCER_CC2P;
//			TIM15->CCER ^= TIM_CCER_CC1P;
////			TIM15->CCER ^= TIM_CCER_CC2P;
//		}
	}
}
