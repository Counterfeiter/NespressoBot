/*
 * NespressoBot.h
 *
 * Created: 01.02.2016 19:17:29
 *  Author: Basti
 */ 


#ifndef NESPRESSOBOT_H_
#define NESPRESSOBOT_H_


#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "suart/swUartConfig.h"

#include <util/delay.h>

// temperature at programming time
#define AIR_TEMPERATURE			22

#define NEW_VERSION

//#define BOT_GEBURTSTAG

#ifdef NEW_VERSION

// use also internal ref
#define ADCMUX_LEDR				(0x02 | (1<<REFS1))
#define ADCMUX_LEDL				(0x03 | (1<<REFS1))

#define GPIO_LEDL				3
#define GPIO_LEDR				4
#define GPIO_SWITCH				2
#define GPIO_SPEAKER			0
#define GPIO_SPEAKER_L			1

#else 

// use also internal ref
#define ADCMUX_LEDR				(0x01 | (1<<REFS1))
#define ADCMUX_LEDL				(0x03 | (1<<REFS1))

#define GPIO_LEDL				3
#define GPIO_LEDR				2
#define GPIO_SWITCH				4
#define GPIO_SPEAKER			0
#define GPIO_SPEAKER_L			1

#endif



#endif /* NESPRESSOBOT_H_ */