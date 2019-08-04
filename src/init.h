#ifndef INIT_H_
#define INIT_H_

#include "stm32f10x.h"

void initComRCC();
void initGPIO();
void initTIM15(/*TIM timInst*/);
void initTIM3();
void initIRPT();

#endif
