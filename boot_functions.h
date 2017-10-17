/*
 * boot_functions.h
 *
 *  Created on: Aug 26, 2017
 *      Author: L01073411
 */

#ifndef BOOT_FUNCTIONS_H_
#define BOOT_FUNCTIONS_H_

#include "ee.h"
#include "ee_irq.h"

/* Defines */
#define LED_D3_CONFIG	TRISAbits.TRISA0
#define LED_D4_CONFIG	TRISAbits.TRISA1
#define LED_D5_CONFIG	TRISAbits.TRISA2
#define LED_D6_CONFIG	TRISAbits.TRISA3
#define LED_D7_CONFIG	TRISAbits.TRISA4
#define LED_D8_CONFIG	TRISAbits.TRISA5
#define LED_D9_CONFIG	TRISAbits.TRISA6
#define LED_D10_CONFIG	TRISAbits.TRISA7

#define LED_D3	LATAbits.LATA0
#define LED_D4	LATAbits.LATA1
#define LED_D5	LATAbits.LATA2
#define LED_D6	LATAbits.LATA3
#define LED_D7	LATAbits.LATA4
#define LED_D8	LATAbits.LATA5
#define LED_D9	LATAbits.LATA6
#define LED_D10	LATAbits.LATA7

#define BUTTON_S3_CONFIG	TRISDbits.TRISD6
#define BUTTON_S4_CONFIG	TRISDbits.TRISD13
#define BUTTON_S5_CONFIG	TRISDbits.TRISA7
#define BUTTON_S6_CONFIG	TRISDbits.TRISD7

#define BUTTON_S3			PORTDbits.RD6
#define BUTTON_S4			PORTDbits.RD13
#define BUTTON_S5			PORTDbits.RA7
#define BUTTON_S6			PORTDbits.RD7

// Function definitions
void clock_setup(void);
void T1_program(void);
void T1_clear(void);
void LCD_init(void);
void put_LCD_initial_message(char message[20]);
void put_LCD_float_value(float value);
void put_LCD_integer_value(int value);


#endif /* BOOT_FUNCTIONS_H_ */
