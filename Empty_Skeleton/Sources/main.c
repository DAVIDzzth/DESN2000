#include "../Headers/stm32f103xe.h"
#include "../LCD/ili9341.h"
#include "../LCD/lcd_hw.h"
#include "../LCD/lcd_grph.h"



void delay (void);

int main(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;
	
	GPIOF->CRL = 0x22222222;
	
	while (1){
		GPIOF->BSRR = 0x000000FF;
		
		delay();
		
		GPIOF->BSRR = 0x00FF0000;
		
		delay();

		
	}
	
	return 0;
}

void delay(void){
	volatile unsigned int count;
	
	for(count = 0;count < 1000000; count++) {
	}
}

