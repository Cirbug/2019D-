#include "delay.h"
void delay_us(uint32_t us)

{
		uint32_t ticks =us * (SystemCoreClock / 1000000);
		uint32_t start 	= SysTick ->VAL;
	 while((start - SysTick ->VAL) < ticks);
}


