/*============================================================================
=
=  Copyright:
=     (C) 2026 Joe Li
= 
=  Description:
=     Screen Controller For APE Board
=		  Can be used with ili9431 screens
=		  initialization file
=
============================================================================*/

#include "stm32f103xe.h"
#include "ili9341.h"
#include "lcd_hw.h"

/*============================================================================
=
= Description:
= 		Initialize the LCD screen as a whole (controller ili9341)
=
============================================================================*/
void lcd_init(void) {
	// FSMC_init turns on RCC for all relevant pins
	FSMC_init();
	
	// turn on backlight and level shifters, enable reset
	RCC->APB2ENR |= RCC_APB2RSTR_IOPBRST_Msk;
	GPIOB->CRL |= 0x2; GPIOB->CRL &= ~(0xD);
	GPIOB->BSRR = (0x1 << 0);
	
	// enable reset FSMC
	GPIOE->CRL |= (0x2 << 24); GPIOE->CRL &= ~(0xD << 24);
	
	// function name explains itself
	LCD_timer_init();
	ili9341_Init();
}



/////////////////////////////////////////////////////////////////////////////
//
//  PRIVATE FUNCTION - NOT INTENDED FOR USERS TO SEE
//		only if you're interested ;)
//
/////////////////////////////////////////////////////////////////////////////

// simple delay
void LCD_Delay(uint32_t delay) {
	LCD_mdelay(delay);
}

// briefly pulse RESET on LCD controller
void LCD_IO_Init(void) {
	GPIOE->BSRR = (0x1 << 22);
	LCD_mdelay(20);
	GPIOE->BSRR = (0x1 << 6);
	LCD_mdelay(20);
}

// read from FSMC protocol - treating lcd as 2 bit address
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) {
	*FSMC_BANK1_DATA = RegValue;
	return *FSMC_BANK1_DATA & ~(0xFFFFFFFF << ReadSize);
}

// write from FSMC protocol - treating lcd as 2 bit address
// with A16 indicating D/C being 0, meaning data transfer
void LCD_IO_WriteData(uint16_t RegValue) {
	*FSMC_BANK1_DATA = RegValue;
	return;
}

// read from FSMC protocol - treating lcd as 2 bit address
// with A16 indicating D/C being 1, meaning control
void LCD_IO_WriteReg(uint8_t Reg) {
	*FSMC_BANK1_REG = Reg;
	return;
}

// initialize timer
void LCD_timer_init(void){
	// TIM5 enable
	// chose timer 5 as it is general purpose
	RCC->APB1ENR |= (1 << 3);
	
	// timer control registers
	TIM5->CR1 &= ~(1 << 4); // count up timer
	TIM5->CR1 &= ~(3 << 5); // edge aligned mode
	TIM5->CR1 &= ~(3 << 8); // no clk div
	
	TIM5->PSC = 48000; // set timer prescaler
}

// timed delay
void LCD_mdelay(unsigned int delay_in_us) {
	TIM5->EGR |= 1;	//reset timer
	TIM5->CR1 |= (1 << 0); // enable clock
	while (TIM5->CNT < delay_in_us); // wait for counter to count
	TIM5->CR1 &= ~(1 << 0); // disable clock
}

// initialise FSMC
void FSMC_init(void){
	// enable clock for GPIO port D, E and G
	RCC->APB2ENR |= RCC_APB2RSTR_IOPDRST_Msk | RCC_APB2RSTR_IOPERST_Msk | RCC_APB2RSTR_IOPGRST_Msk;
	RCC->AHBENR |= (1 << 8);
	
	// Set GPIO Alternate Function for FSMC pins
	
	GPIOD->CRL |= 0xB0BB00BB; GPIOD->CRL &= 0xBFBBFFBB;
	GPIOD->CRH |= 0xBB00BBBB; GPIOD->CRH &= 0xBBFFBBBB;
	
	GPIOE->CRL |= 0xB0000000; GPIOE->CRL &= 0xBFFFFFFF;
	GPIOE->CRH = 0xBBBBBBBB;

	GPIOG->CRH |= 0x00B00000; GPIOG->CRH &= 0xFFBFFFFF;
	
	// stm32f103xe.h somehow merged the BCR and BTR register
	FSMC_Bank1->BTCR[0] = 0x000010D3;
	FSMC_Bank1->BTCR[1] = 0x00100302;
}
