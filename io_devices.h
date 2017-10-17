/*
 * io_devices.h
 *
 *  Created on: Oct 17, 2017
 *      Author: Luisa
 */

#ifndef IO_DEVICES_H_
#define IO_DEVICES_H_

void ADC_int(void);
void PWM_init(void);
void Serial_Init(void);
void Serial_Send(void);
void Serial_Send_Frame(void);

#endif /* IO_DEVICES_H_ */
