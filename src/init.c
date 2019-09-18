#include "init.h"

void initComRCC() {
// General settings to set SYSCLK
	//Reset RCC System
	RCC->CR |= RCC_CR_HSION;
	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
	RCC->CFGR &= (uint32_t)0xF8FF0000;
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
	RCC->CFGR &= (uint32_t)0xFF80FFFF;
	//Set PLL multiplication
	RCC->CFGR |= RCC_CFGR_PLLMULL_2;
	/* Set CFGR2 register - HSE divider*/
	RCC->CFGR2 |= 0x00000000; // PLL is divided by 2
	RCC->CFGR |= RCC_CFGR_MCO_2; // SYSCLK as an output on PA8
//	RCC->CFGR |= RCC_CFGR_MCO; // PLL/2 as an output on PA8
	// Set HSE through PLL as a SYSCLKstm32
	RCC->CFGR |= RCC_CFGR_SW_1; // SYSCLK is fed by PLL
	RCC->CFGR |= RCC_CFGR_PLLSRC_PREDIV1; // PLL is fed by HSE after PREDIV
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;

//	RCC->CFGR |= RCC_CFGR_APB2
	// Enable HSE and then PLL
	RCC->CR |= RCC_CR_HSEON; // 24MHz HSE ON
	RCC->CR |= RCC_CR_PLLON; // PLL ON

// Enable clocking of peripheral
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // GPIO A port
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // GPIO B port
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // GPIO C port
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1
	// Deinitialization of TIM15
	RCC->APB2RSTR |= RCC_APB2ENR_TIM15EN;
	RCC->APB2RSTR &= ~RCC_APB2ENR_TIM15EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // TIM15

	RCC->APB1RSTR |= RCC_APB1ENR_TIM3EN;
	RCC->APB1RSTR &= ~RCC_APB1ENR_TIM3EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // TIM15
}

void initGPIO() {
// Port A
	GPIOA->CRH |= GPIO_CRH_MODE8; // Set PA8 pin speed to 50 MHz
	// Set PA8 pin as AF output push-pull for MCO
	GPIOA->CRH &= ~GPIO_CRH_CNF8;
	GPIOA->CRH |= GPIO_CRH_CNF8_1;

	GPIOA->CRL |= GPIO_CRL_MODE2; // Set PA2 pin speed to 50 MHz
	// Set PA2 pin as AF output push-pull for TIM15
	GPIOA->CRL &= ~GPIO_CRL_CNF2;
	GPIOA->CRL |= GPIO_CRL_CNF2_1;

	GPIOA->CRL |= GPIO_CRL_MODE3; // Set PA3 pin speed to 50 MHz
	// Set PA3 pin as AF output push-pull for TIM15
	GPIOA->CRL &= ~GPIO_CRL_CNF3;
	GPIOA->CRL |= GPIO_CRL_CNF3_1;

	GPIOA->CRL |= GPIO_CRL_MODE6; // Set PA6 pin speed to 50 MHz
	// Set PA6 pin as AF output push-pull for TIM3
	GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL |= GPIO_CRL_CNF6_1;

	GPIOA->CRL |= GPIO_CRL_MODE7; // Set PA7 pin speed to 50 MHz
	// Set PA7 pin as AF output push-pull for TIM3
	GPIOA->CRL &= ~GPIO_CRL_CNF7;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;

	GPIOA->CRL &= ~GPIO_CRL_MODE0; // Set PA0 pin as input
	// Set PA0 pin to input with pull-up/pull-down
	GPIOA->CRL &= ~GPIO_CRL_CNF0;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;

	GPIOA->CRL &= ~GPIO_CRL_MODE1; // Set PB3 pin as input
	// Set PB3 pin to input with pull-up/pull-down
	GPIOA->CRL &= ~GPIO_CRL_CNF1;
	GPIOA->CRL |= GPIO_CRL_CNF1_1;

//Port B
	GPIOB->CRL &= ~GPIO_CRL_MODE4; // Set PB4 pin as input
	// Set PB4 pin to input with pull-up/pull-down
	GPIOB->CRL &= ~GPIO_CRL_CNF4;
	GPIOB->CRL |= GPIO_CRL_CNF4_1;

//Port C
	GPIOC->CRL &= ~GPIO_CRL_MODE0; // Set PC0 pin as input for ADC
	// Set PC0 pin to analog input
	GPIOC->CRL &= ~GPIO_CRL_CNF0;
//	GPIOC->CRL |= GPIO_CRL_CNF0;

	GPIOC->CRH &= ~GPIO_CRH_MODE9; // Set PC9 pin as output
	GPIOC->CRH |= GPIO_CRH_MODE9_1; // Output speed - 2MHz
	GPIOC->CRH &= ~GPIO_CRH_CNF9;	// General purpose push-pull

// EXTI0 line is fed by PA
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;
// EXTI1 line is fed by PA
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA;
// EXTI2 line is fed by PB
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PB;
}

void initTIM3() {
// General settings to set TimeBase parameters
	// Set TIM3 as a master
//	TIM3->CR2 &= ~TIM_CR2_MMS;
//	TIM3->CR2 |= TIM_CR2_MMS_1;
//	TIM15->CR1 |= TIM_CR1_DIR; // Upcount mode
	TIM3->BDTR |= TIM_BDTR_MOE;
	TIM3->ARR = 0xFFFF; 	   // TIM15 auto reload register is set to 65535
	TIM3->PSC = 0x4;
	TIM3->RCR = 0;
	// Enable TIM3
//	TIM3->CR1 |= TIM_CR1_CEN;
//Channel 1
	TIM3->CCER &= ~TIM_CCER_CC1E; // Disable channel 1
	/* Reset the Output Compare Mode Bits */
	TIM3->CCMR1 &=	~TIM_CCMR1_OC1M;
	TIM3->CCMR1 &=	~TIM_CCMR1_CC1S;
	TIM3->CCER &= ~TIM_CCER_CC1E;
	TIM3->CCER &= ~TIM_CCER_CC1NP;

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // Set output compare toggle mode
//	TIM3->CCER &= ~TIM_CCER_CC1P; // Set high polarity
//	TIM3->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable

	TIM3->CCR1 |= TIM_CCR1_CCR1; // Set 90 deg shift
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload register

// Channel 2
	TIM3->CCER &= ~TIM_CCER_CC2E; // Disable channel 2
	/* Reset the Output Compare Mode Bits */
	TIM3->CCMR1 &=	~TIM_CCMR1_OC2M;
	TIM3->CCMR1 &=	~TIM_CCMR1_CC2S;
	TIM3->CCER &= ~TIM_CCER_CC2E;
	TIM3->CCER &= ~TIM_CCER_CC2NP;

	TIM3->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1; // Set output compare toggle mode
//	TIM3->CCER |= TIM_CCER_CC2P; // Set low polarity
//	TIM3->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable

	TIM3->CCR2 |= TIM_CCR1_CCR1/2; // Set 45 deg shift
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload register
}

void initTIM15() {
// General settings to set TimeBase parameters
	//Set TIM15 as a slave
//	TIM15->SMCR &= ~TIM_SMCR_TS;
	// Configure a trigger mode - counter starts at a rising edge of TRIGI
//	TIM15->SMCR &= ~TIM_SMCR_SMS;
//	TIM15->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
//	TIM15->CR1 |= TIM_CR1_DIR; // Upcount mode
	TIM15->BDTR |= TIM_BDTR_MOE;
	TIM15->ARR = 0xFFFF; 	   // TIM15 auto reload register is set to 65535
	TIM15->PSC = 0x4;
	TIM15->RCR = 0;

	// Enable TIM15
//	TIM15->CR1 |= TIM_CR1_CEN;
	// PWM settings
//Channel 1
	TIM15->CCER &= ~TIM_CCER_CC1E; // Disable channel 1
	/* Reset the Output Compare Mode Bits */
	TIM15->CCMR1 &=	~TIM_CCMR1_OC1M;
	TIM15->CCMR1 &=	~TIM_CCMR1_CC1S;
	TIM15->CCER &= ~TIM_CCER_CC1E;
	TIM15->CCER &= ~TIM_CCER_CC1NP;

	TIM15->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // Set output compare toggle mode
	TIM15->CCER |= TIM_CCER_CC1P; // Set low polarity
//	TIM15->CCER |= TIM_CCER_CC1E; // Output state channel 1 enable

	TIM15->CCR1 |= TIM_CCR1_CCR1; // Set duty cycle to 50%
	TIM15->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload register

// Channel 2
	TIM15->CCER &= ~TIM_CCER_CC2E; // Disable channel 2
	/* Reset the Output Compare Mode Bits */
	TIM15->CCMR1 &=	~TIM_CCMR1_OC2M;
	TIM15->CCMR1 &=	~TIM_CCMR1_CC2S;
	TIM15->CCER &= ~TIM_CCER_CC2E;
	TIM15->CCER &= ~TIM_CCER_CC2NP;

	TIM15->CCMR1 |= TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1; // Set output compare toggle mode
	TIM15->CCER &= ~TIM_CCER_CC2P; // Set high polarity
//	TIM15->CCER |= TIM_CCER_CC2E; // Output state channel 2 enable

	TIM15->CCR2 |= TIM_CCR1_CCR1/2; // Set duty cycle to 50%
	TIM15->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload register
}

void initADC1() {

	ADC1->SMPR1 = ADC_SMPR1_SMP10;

	ADC1->CR1 |= ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_3;
	ADC1->CR1 |= ADC_CR1_AWDEN;
	ADC1->CR1 |= ADC_CR1_AWDIE;

	// Set thresholds values for watchdog
	ADC1->HTR = 2200; // Current - ADC samples ratio = ~ 1.78
	ADC1->LTR = 0;

	ADC1->CR2 |= ADC_CR2_ADON;
	// Calibration
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL) {
		__NOP; // waiting for end of calibration
	}

	ADC1->SQR3 = 10;
	// Start conversion
	ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_CONT;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void initIRPT() {
	// Enable NVIC interrupt for ADC1
	NVIC_EnableIRQ(ADC1_IRQn);
	// Enable NVIC interrupt for EXTI0
	NVIC_EnableIRQ(EXTI0_IRQn);
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;
	// Enable NVIC interrupt for EXTI1
	NVIC_EnableIRQ(EXTI1_IRQn);
	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI->FTSR |= EXTI_FTSR_TR1;
	// Enable NVIC interrupt for EXTI4
	NVIC_EnableIRQ(EXTI2_IRQn);
	EXTI->IMR |= EXTI_IMR_MR4;
	EXTI->FTSR |= EXTI_FTSR_TR4;
}
