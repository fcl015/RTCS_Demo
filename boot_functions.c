/*
 * boot_functions.c
 *
 *  Created on: Aug 26, 2017
 *      Author: L01073411
 */

#include "ee.h"
#include "ee_irq.h"
#include "boot_functions.h"


/* Clock set-up for 40MIPS */
void clock_setup(void)
{
	/* PLL Configuration */
	PLLFBD=38; 				// M=40
	CLKDIVbits.PLLPOST=0; 	// N1=2
	CLKDIVbits.PLLPRE=0; 	// N2=2
	OSCTUN=0; 				// FRC clock use
	RCONbits.SWDTEN=0; 		//watchdog timer disable
	while(OSCCONbits.LOCK!=1); //wait for PLL LOCK

}

/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;		/* Stops the Timer1 and reset control reg	*/
	TMR1  = 0;		/* Clear contents of the timer register	*/
	PR1   = 0x9c40;		/* PR1=40000 Load the Period register with the value of 1ms	*/
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1		*/
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
						* and clock source set to the internal
						* instruction cycle			*/
}

/* Clear the Timer1 interrupt status flag */
void T1_clear(void)
{
	IFS0bits.T1IF = 0;
}

/* Initialize LCD and wait until it is ready */
void LCD_init()
{
	EE_lcd_init();
	EE_lcd_clear();
	while(EE_lcd_busy());
}

/* Writes an initial message in the LCD display first row */
void put_LCD_initial_message()
{
	EE_lcd_goto( 0, 0 );

	EE_lcd_putc('D');
	EE_lcd_putc('a');
	EE_lcd_putc('n');
	EE_lcd_putc('i');
	EE_lcd_putc('e');
	EE_lcd_putc('l');
	EE_lcd_putc('e');
	EE_lcd_putc('s');
	EE_lcd_putc(' ');
	EE_lcd_putc('Y');
	EE_lcd_putc(' ');
	EE_lcd_putc('R');
	EE_lcd_putc('o');
	EE_lcd_putc('j');
	EE_lcd_putc('a');
	EE_lcd_putc('s');

}

/* Writes a float value from 00.00 to 99.99 in the LCD second row */
void put_LCD_float_value(float value)
{
	unsigned int digit1,digit2,digit3,digit4;

	EE_lcd_line2();

	EE_lcd_putc('V');
	EE_lcd_putc('a');
	EE_lcd_putc('l');
	EE_lcd_putc('u');
	EE_lcd_putc('e');
	EE_lcd_putc('=');

	digit1=(unsigned int)(value/10);
	EE_lcd_putc(digit1+'0');

	value=(float)fmodf(value,10.0);
	digit2=(unsigned int)(value/1);
	EE_lcd_putc(digit2+'0');

	EE_lcd_putc('.');

	value=(float)fmodf(value,1.0);
	digit3=(unsigned int)(value*10);
	EE_lcd_putc(digit3+'0');

	value=(float)fmodf(value,0.1);
	digit4=(unsigned int)(value*100);
	EE_lcd_putc(digit4+'0');

	EE_lcd_putc('%');
}

/* Writes a integer value from 0 to 9999 in the LCD second row */
void put_LCD_integer_value(int value)
{
	unsigned int digit1,digit2,digit3,digit4;
	float f_value;

	EE_lcd_goto( 0, 0 );

	EE_lcd_putc('R');
	EE_lcd_putc('a');
	EE_lcd_putc('w');
	EE_lcd_putc(' ');
	EE_lcd_putc('V');
	EE_lcd_putc('a');
	EE_lcd_putc('l');
	EE_lcd_putc('u');
	EE_lcd_putc('e');
	EE_lcd_putc('=');

	f_value=(float)value;

	digit1=(unsigned int)(f_value/1000);
	EE_lcd_putc(digit1+'0');

	f_value=(float)fmodf(f_value,1000.0);
	digit2=(unsigned int)(f_value/100);
	EE_lcd_putc(digit2+'0');

	f_value=(float)fmodf(f_value,100.0);
	digit3=(unsigned int)(f_value/10);
	EE_lcd_putc(digit3+'0');

	f_value=(float)fmodf(f_value,10.0);
	digit4=(unsigned int)(f_value/1);
	EE_lcd_putc(digit4+'0');

}

