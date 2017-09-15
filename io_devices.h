/*
 * io_devices.h
 *
 *  Created on: Sep 14, 2017
 *      Author: Daniel
 */

#ifndef IO_DEVICES_H_
#define IO_DEVICES_H_

void ADC1_init(void);
void PWM_init(void);
void Serial_Init(void);
int Serial_Send(unsigned char data);
void Serial_Send_Frame(unsigned char *ch, unsigned char len);

#endif /* IO_DEVICES_H_ */


