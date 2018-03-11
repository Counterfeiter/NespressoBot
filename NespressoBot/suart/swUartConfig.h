/********************************************************************************

This file contains configuration information for the half duplex software UART
implementation using Timer0 and external interrupt 0.

All of TX_PIN, TXDDR and TXPORT can be defined here.  If none are defined here,
they will default to the values in swUart.c.

Note that the RX_PIN must be the external interrupt 0 pin on your AVR of choice.
RX_PIN, RXDDR, RXPORT and RX_PIN are defined in swUart.c.  They cannot be defined
here.

The software UART functions can be compiled for TX_ONLY or RX_ONLY by
uncommenting one or the other defines below.

Application note:
  AVR304: Half Duplex Interrupt Driven Software UART

Author:
  Atmel Corporation: http://www.atmel.com \n
  Support email: avr@atmel.com

--------------------------------------------------------------------------------

  Change Activity:

      Date       Description
     ------      -------------
    18 Mar 2011  Separated configuration data from swUart.h.

********************************************************************************/



#ifndef _SW_UART_CONFIG_H_
#define _SW_UART_CONFIG_H_



/********************************************************************************

                                    defines

********************************************************************************/



// optional: select TX_ONLY or RX_ONLY by uncommenting one of the following
// defines

#define TX_ONLY
//#define RX_ONLY
#define F_CPU 1000000UL



// desired baudrate

#define BAUDRATE 1200UL



// timer/counter prescaling: select 1, 8 or 64
// Smaller values will give less baud rate error.  Select the smallest value
// that doesn't give an error.  Which value can be selected depends on F_CPU
// and BAUDRATE.

#define TIMER_PRESCALING 64



// optional: define TX PIN, PORT and DDR (define all or none)

//#define TX_PIN PD3
//#define TXPORT PORTD
//#define TXDDR  DDRD



#endif // ifndef _SW_UART_CONFIG_H_
