/*============================================================================
=
=  Copyright:
=     (C) 2026 Joe Li
=
============================================================================*/
 
#include "ili9341.h"

/* User Functions */
void lcd_init(void);


/////////////////////////////////////////////////////////////////////////////
//
//  PRIVATE SECTION
//
/////////////////////////////////////////////////////////////////////////////

/* Private Section */
void FSMC_init(void);
void LCD_timer_init(void);
void LCD_mdelay(unsigned int delay_in_us);

/* Microcontroller FSMC registers */
#define FSMC_BANK1_REG 					 ((uint16_t *) 0x60000000)	
#define FSMC_BANK1_DATA 				 ((uint16_t *) 0x60020000)
