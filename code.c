/*
 * code.c
 *
 *  Created on: 25/08/2017
 *      Author: Rafael Camilo Lozoya Gámez
 */


/* Include files for Erika kernel functions */
#include "ee.h"
#include "ee_irq.h"

/* Include files for my functions */
#include "boot_functions.h"
#include "io_devices.h"


// MIPS40 - Run CPU at maximum speed 40MIPS (25ns), oscillator with PLL at 80Mhz
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);
// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystanl
_FOSC(OSCIOFNC_ON & POSCMD_XT);
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);

/* Global Variables */
static unsigned int my_time=0;



/******************************************************************************************
 * Función:	ADC1_init()						     										  *
 * Descripción:	Configure ADC1.			 		          								  *
 ******************************************************************************************/
void ADC1_init(void)
{
	AD1CON1bits.ADON  = 0;// ADC Operating Mode bit. Turn off the A/D converter

	/*ADC Configuration*/
	AD1PCFGL = 0xFFFF;		//ADC1 Port Configuration Register Low
	AD1PCFGH = 0xFFFF;		//ADC1 Port Configuration Register High

	AD1PCFGLbits.PCFG5=0;   //Potentiometer input RB5/AN5

	AD1CON2bits.VCFG = 0;    /*Converter Voltage Reference Configuration bits
				(ADRef+=AVdd, ADRef-=AVss)*/
	AD1CON3bits.ADCS = 63;   /* ADC Conversion Clock Select bits
			     	*(Tad = Tcy*(ADCS+1) = (1/40000000)*64 = 1.6us)
				*Tcy=Instruction Cycle Time=40MIPS */
	AD1CON2bits.CHPS = 0;	/* Selects Channels Utilized bits, When AD12B = 1,
				 * CHPS<1:0> is: U-0, Unimplemented, Read as ‘0’ */
	AD1CON1bits.SSRC = 7;/*Sample Clock Source Select bits:
		  	111 = Internal counter ends sampling and starts
			  	conversion (auto-convert) */

	AD1CON3bits.SAMC = 31;	// Auto Sample Time bits. (31*Tad = 49.6us)
	AD1CON1bits.FORM = 0;	// Data Output Format bits. Integer
				/* For 12-bit operation:
				   00 = Integer
				   (DOUT = 0000 dddd dddd dddd)*/

	AD1CON1bits.AD12B = 1;	/* Operation Mode bit:
				   0 = 10 bit
				   1 = 12 bit*/
	AD1CON1bits.ASAM  = 0;	/* ADC Sample Auto-Start bit:
			       1 = Sampling begins immediately after last
			       conversion. SAMP bit is auto-set.
			   0 = Sampling begins when SAMP bit is set*/
	AD1CHS0bits.CH0NA = 0;	// MUXA  -Ve input selection (Vref-) for CH0.

	AD1CON1bits.ADON  = 1;	// ADC Operating Mode bit. Turn on A/D converter
}

/******************************************************************************************
 * Función:	PWM_init()						     										  *
 * Descripción:	Configure PWM			 		          								  *
 ******************************************************************************************/
void PWM_init(void)
{
	// Initialize Output Compare Module
	OC1CONbits.OCM = 0b000; 	// Disable Output Compare Module
	OC1R = 0x0; 				// Write the duty cycle for the first PWM pulse
	OC1RS = 0x0; 				// Write the duty cycle for the second PWM pulse
	OC1CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
	OC1R = 0x0; 				// Load the Compare Register Value
	OC1CONbits.OCM = 0b110; 	// Select the Output Compare mode
	// Initialize and enable Timer2
	T2CONbits.TON = 0; 			// Disable Timer
	T2CONbits.TCS = 0; 			// Select internal instruction cycle clock
	T2CONbits.TGATE = 0; 		// Disable Gated Timer mode
	T2CONbits.TCKPS = 0b00; 	// Select 1:1 Preescaler
	TMR2 = 0x00; 				// Clear timer register
	PR2 = 0x0FFF; 				// Load the period value
								// 4MIPS 	=> PWM_Freq=977Hz, Resolution=12bits
								// 40MIPS	=> PWM_Freq=9.77KHz, Resolution=12bits
	T2CONbits.TON = 1; 			// Start Timer
}

/******************************************************************************************
 * Función:	Serial_init()						     										  *
 * Descripción:	Configura Serial Port			 		          								  *
 ******************************************************************************************/
void Serial_Init(void)
{
	/* Stop UART port */
	U2MODEbits.UARTEN = 0;

	/* Disable Interrupts */
	IEC1bits.U2RXIE = 0;
	IEC1bits.U2TXIE = 0;

	/* Clear Interrupt flag bits */
	IFS1bits.U2RXIF = 0;
	IFS1bits.U2TXIF = 0;

	/* Set IO pins */
	TRISFbits.TRISF12 = 0;  // CTS Output
	TRISFbits.TRISF13 = 0;  // RTS Output
	TRISFbits.TRISF5 = 0;   // TX Output
	TRISFbits.TRISF4 = 1;   // RX Input

	/* baud rate */
	U2MODEbits.BRGH = 0;
	U2BRG  = 21; // 42 -> 57600 baud rate / 21-> 115200 baud rate

	/* Operation settings and start port */
	U2MODE = 0;
	U2MODEbits.UEN = 0; //2
	U2MODEbits.UARTEN = 1;

	/* TX & RX interrupt modes */
	U2STA = 0;
	U2STAbits.UTXEN=1;
}

/******************************************************************************************
 * Send one byte						     										  *
 ******************************************************************************************/
int Serial_Send(unsigned char data)
{
	while (U2STAbits.UTXBF);
	U2TXREG = data;
	while(!U2STAbits.TRMT);
	return 0;
}

/******************************************************************************************
 * Send a group of bytes						     										  *
 ******************************************************************************************/
void Serial_Send_Frame(unsigned char *ch, unsigned char len)
{
   unsigned char i;

   for (i = 0; i < len; i++) {
	Serial_Send(*(ch++));
   }
}





/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference
 */
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	T1_clear();
	/* Redirect PWM output OC1/RD0 to D4 led RA1 */
	LED_D4=PORTDbits.RD0;
	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}



/******************************************************************************************
 * TASKS					     										  *
 ******************************************************************************************/

TASK(Task1)
{
	/* Blink leds every 1 second */
	LED_D3^=1;				// Led D3
	my_time++;				// Increment global variable my_time (every second)
}

TASK(Task2)
{
	float value1;

	AD1CHS0 = 5;   					// Channel 5
	AD1CON1bits.SAMP = 1;  			// Start conversion
	while(!IFS0bits.AD1IF);			// Wait till the EOC
	IFS0bits.AD1IF = 0;    			// reset ADC interrupt flag
	value1=(ADC1BUF0/4096.0)*100.0;  //scale to relative percentage
	put_LCD_float_value(value1);     // Display percentage value
	put_LCD_integer_value(ADC1BUF0);// Display raw value
	OC1RS=ADC1BUF0;        			//Send ADC read to PWM output
}

TASK(Task3)
{
	LED_D5=~BUTTON_S3;	// Button S3 -> Activates Led D5
}

TASK(Task4)
{
	unsigned char BufferOut[10];

	// Send data to PC via serial port
	BufferOut[0]='!';
	BufferOut[1]=(char)(my_time>>8);
	BufferOut[2]=(char)my_time;
	BufferOut[3]=0xFF;
	BufferOut[4]=0xFF;
	BufferOut[5]=0x0A;
	Serial_Send_Frame(BufferOut,6);
}


//******************************************************************************************
// Main program
//******************************************************************************************
// Execute four tasks:
//		- Blink a led every second
//		- Read analog valur from pot and display it in the LCD
//		- Press button and activate led
//		- Transmit data through the serial port
//******************************************************************************************
int main(void)
{

	/* Clock set-up and program Timer 1 */
	clock_setup();
	T1_program();

	/* Init leds as outputs*/
	LED_D3_CONFIG=0;   	// Led D3
	LED_D4_CONFIG=0;	// Led D4
	LED_D5_CONFIG=0;	// Led D5

	/* Init push button as input*/
	BUTTON_S3_CONFIG=1;		// Button S3

	/* Init LCD */
	LCD_init();

	/* Modules init */
	ADC1_init();
	PWM_init();
	Serial_Init();

	/* Program cyclic alarms which will fire after an initial offset, and after that periodically */
	SetRelAlarm(Alarm1, 1000,  1000);
	SetRelAlarm(Alarm2, 1100,  100);
	SetRelAlarm(Alarm3, 1200,  100);
	SetRelAlarm(Alarm4, 1300,  5000);

	 /* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}
