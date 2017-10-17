/*
 * io_devices.h
 *
 *  Created on: 16/10/2017
 *      Author: casa
 */

void ADC1_init(void);
void PWM_init(void);
void Serial_Init(void);
int Serial_Send(unsigned char data);
void Serial_Send_Frame(unsigned char *ch, unsigned char len);

#ifndef IO_DEVICES_H_
#define IO_DEVICES_H_



#endif /* IO_DEVICES_H_ */
