#ifndef __SOLENOID_H
#define __SOLENOID_H

#include "includes.h"

extern uint16_t solenoid_flag;

void solenoid_init(uint8_t usart_channel);
void solenoid_on(uint8_t usart_channel, uint8_t cmd);
#endif
