/********************************************************************************

Half duplex software UART implementation using Timer0 and external interrupt 0.

Note that the RX_PIN must be the external interrupt 0 pin on your AVR of choice.
The TX_PIN can be chosen to be any suitable pin.  If other operating voltages
and/or temperatures than 5 Volts and 25 Degrees Celsius are desired, consider
calibrating the internal oscillator.

Application note:
  AVR304: Half Duplex Interrupt Driven Software UART

Author:
  Atmel Corporation: http://www.atmel.com \n
  Support email: avr@atmel.com

--------------------------------------------------------------------------------

  Change Activity:

      Date       Description
     ------      -------------
    29 Mar 2010  Created from AVR304 and ported to WINAVR (AVR GCC).
                 Enhanced baudrate support.
                 Added support for various processor speeds (F_CPU).
                 Added support for ATtiny25, ATtiny45 and ATtiny85.
    19 Feb 2011  Broke up into seperate .c and .h files.
    11 Mar 2011  Added txBusy function.
    15 Mar 2011  Added conditional compilation for TX_ONLY or RX_ONLY.
    18 Mar 2011  Moved configuration data into swUartConfig.h.
    24 Mar 2011  Changed #include "swUartConfig.h" to <swUartConfig.h> so
                 the preprocessor doesn't first look in the same directory
                 as this file.

********************************************************************************/



#ifndef _SW_UART_H_
#define _SW_UART_H_



/********************************************************************************

                                    includes

********************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "swUartConfig.h"

// Type defined enumeration holding software UART's state.
typedef enum
{
	IDLE,                                       // Idle state, both transmit and
	// receive possible.
	#ifndef RX_ONLY
	TRANSMIT,                                   // Transmitting byte.
	TRANSMIT_STOP_BIT                           // Transmitting stop bit.
	#  ifndef TX_ONLY
	,
	#  endif
	#endif
	#ifndef TX_ONLY
	RECEIVE,                                    // Receiving byte.
	DATA_PENDING                                // Byte received and ready to
	#endif
} AsynchronousStates_t;



AsynchronousStates_t getState();
void setState(AsynchronousStates_t state_l);

void disbaleRXInt();
void enableRXInt();

/********************************************************************************

				initSoftwareUart

Initialize the software UART.

This function will set up pins to transmit and receive on.  Control of Timer0 and
External interrupt 0.

********************************************************************************/

void initSoftwareUart( void );



#ifndef RX_ONLY

/********************************************************************************

                                    putChar

Send an unsigned char

This function sends an unsigned char on the TX_PIN using the Timer0 ISR.

Notes:

1. initSoftwareUart( void ) must be called in advance.

2. An unread Rx byte will be lost.

********************************************************************************/

int putChar( char  c);

#endif // ifndef RX_ONLY



#ifndef TX_ONLY

/********************************************************************************

                                    getChar

Receive an unsigned char

This function receives an unsigned char on the RX_PIN using the Timer0 ISR.

Notes:

1. initSoftwareUart( void ) must be called in advance.

2. Only a single Rx byte is buffered.  If an input byte is not read before the
   next START bit, it will be lost.

********************************************************************************/

int getChar();

#endif // ifndef TX_ONLY



#ifndef RX_ONLY

/********************************************************************************

                                     txBusy

Tests for Tx busy.

Returns true if Tx in process; false otherwise.

********************************************************************************/

bool txBusy( void );

#endif // ifndef RX_ONLY



#endif // define _SW_UART_H_
