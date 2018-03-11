/********************************************************************************

Half duplex software UART implementation using Timer0 and external interrupt 0.

Note that the RX_PIN must be the external interrupt 0 pin on your AVR of choice.

The TX_PIN can be chosen to be any suitable pin.  If you don't want to use the
vaule here, define TX_PIN, TXDDR and TXPORT in swUartConfig.h.

If the internal oscillator is being used and other operating voltages and/or
temperatures than 5 Volts and 25 Degrees Celsius are desired, consider
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
    11 Mar 2011  Added support for ATtiny24, ATtiny44 and ATtiny84.
                 Allow seperate ports for TX and RX pins.
                 Added txBusy function.
                 Added test for state equal to DATA_PENDING in addition to IDLE
                 in the putChar function so it doesn't hang if any Rx data occurs
                 while transmitting.
    15 Mar 2011  Added conditional compilation for TX_ONLY or RX_ONLY.
    18 Mar 2011  Added setting OCR and CLEAR_TIMER_INTERRUPT to putChar (missing
                 from original AVR304 code).  Also removed duplicate
                 CLEAR_TX_PIN (in original AVR304 code).

********************************************************************************/



/********************************************************************************

                                    includes

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "swUart.h"
#include "../NespressoBot.h"


/********************************************************************************

                               defines

********************************************************************************/

#ifdef TX_ONLY
#  ifdef RX_ONLY
#    error TX_ONLY and RX_ONLY cannot both be defined!
#  endif
#endif

#if TIMER_PRESCALING == 1
# define CLOCK_SELECT ( ( 0 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 ) )
#elif TIMER_PRESCALING == 8
# define CLOCK_SELECT ( ( 0 << CS02 ) | ( 1 << CS01 ) | ( 0 << CS00 ) )
#elif TIMER_PRESCALING == 64
# define CLOCK_SELECT ( ( 0 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) )
#elif TIMER_PRESCALING == 256
# define CLOCK_SELECT ( ( 1 << CS02 ) | ( 0 << CS01 ) | ( 0 << CS00 ) )
#elif TIMER_PRESCALING == 1024
# define CLOCK_SELECT ( ( 1 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 ) )
#else
# error Unknown TIMER_PRESCALING value
#endif

#define CLOCK_SELECT_MASK ( ( 1 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 ) )



// determine timer values for desired baudrate

// clock ticks to wait one bit period
#define TICKS2WAITONE ( \
  ( ( ( ( 2 * F_CPU ) / ( BAUDRATE * TIMER_PRESCALING ) ) + 1 ) / 2 ) \
)

// clock ticks to wait one and a half bit periods
#define TICKS2WAITONE_HALF ( \
  ( ( ( ( 3 * F_CPU ) / ( BAUDRATE * TIMER_PRESCALING ) ) + 1 ) / 2 ) \
)

#if ( TICKS2WAITONE_HALF > 256 )
# error TIMER_PRESCALING value is too large
#endif



#define INTERRUPT_EXEC_CYCLES 24UL          // CPU cycles in INT0 interrupt
                                            // handler until TCNT0 is loaded

#define INTERRUPT_EXEC_CYCLES_COUNT ( \
  ( ( ( 2 * INTERRUPT_EXEC_CYCLES ) / TIMER_PRESCALING ) + 1 ) / 2 \
)



// I/O, timer and interrupt specific defines.

#if  ( defined( __AVR_ATmega16__ ) || defined( __AVR_ATmega32__ ) )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK |= ( 1 << OCIE0 ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK &= ~( 1 << OCIE0 ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR |= ( ( 1 << OCF0) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   ( GICR |= ( 1 << INT0 ) )
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( GICR &= ~( 1 << INT0 ) )
# ifndef TX_PIN
#   define TX_PIN         PD3               // Transmit data pin
#   define TXDDR          DDRD
#   define TXPORT         PORTD
# endif
# define RX_PIN           PD2               // Receive data pin, must be INT0
# define RXDDR            DDRD
# define RXPORT           PORTD
# define RXPIN            PIND
# define TCCR             TCCR0             // Timer/Counter Control Register
# define TCCR_P           TCCR0             // Timer/Counter Control (Prescaler)
                                            // Register
# define OCR              OCR0              // Output Compare Register
# define EXT_IFR          GIFR              // External Interrupt Flag Register
# define EXT_ICR          MCUCR             // External Interrupt Control
                                            // Register
# define TIMER_COMP_VECT  TIMER0_COMP_vect  // Timer Compare Interrupt Vector

#elif defined( __AVR_ATmega128__ )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK |= ( 1 << OCIE0 ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK &= ~( 1 << OCIE0 ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR |= ( ( 1 << OCF0 ) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   ( EIMSK |= ( 1 << INT0 ) )
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( EIMSK &= ~( 1 << INT0 ) )
# ifndef TX_PIN
#   define TX_PIN         PD1               // Transmit data pin
#   define TXDDR          DDRD
#   define TXPORT         PORTD
# endif
# define RX_PIN           PD0               // Receive data pin, must be INT0
# define RXDDR            DDRD
# define RXPORT           PORTD
# define RXPIN            PIND
# define TCCR             TCCR0             // Timer/Counter Control Register
# define TCCR_P           TCCR0             // Timer/Counter Control (Prescaler)
                                            // Register
# define OCR              OCR0              // Output Compare Register
# define EXT_IFR          EIFR              // External Interrupt Flag Register
# define EXT_ICR          EICRA             // External Interrupt Control
                                            // Register
# define TIMER_COMP_VECT  TIMER0_COMP_vect  // Timer Compare Interrupt Vector

#elif defined( __AVR_ATmega169__ )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK0 |= ( 1 << OCIE0A ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK0 &= ~( 1 << OCIE0A ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR0 |= ( ( 1 << OCF0A ) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   ( EIMSK |= ( 1 << INT0 ) )
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( EIMSK &= ~( 1 << INT0 ) )
# ifndef TX_PIN
#   define TX_PIN         PD3               // Transmit data pin
#   define TXDDR          DDRD
#   define TXPORT         PORTD
# endif
# define RX_PIN           PD1               // Receive data pin, must be INT0
# define RXDDR            DDRD
# define RXPORT           PORTD
# define RXPIN            PIND
# define TCCR             TCCR0A            // Timer/Counter Control Register
# define TCCR_P           TCCR0A            // Timer/Counter Control (Prescaler)
                                            // Register
# define OCR              OCR0A             // Output Compare Register
# define EXT_IFR          EIFR              // External Interrupt Flag Register
# define EXT_ICR          EICRA             // External Interrupt Control
                                            // Register
# define TIMER_COMP_VECT  TIMER0_COMP_vect  // Timer Compare Interrupt Vector

#elif ( defined( __AVR_ATmega48__ ) || defined( __AVR_ATmega88__ ) )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK0 |= ( 1 << OCIE0A ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK0 &= ~( 1 << OCIE0A ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR0 |= ( ( 1 << OCF0A ) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   ( EIMSK |= ( 1 << INT0 ) )
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( EIMSK &= ~( 1 << INT0 ) )
# ifndef TX_PIN
#   define TX_PIN         PD3                // Transmit data pin
#   define TXDDR          DDRD
#   define TXPORT         PORTD
# endif
# define RX_PIN           PD2                // Receive data pin, must be INT0
# define RXDDR            DDRD
# define RXPORT           PORTD
# define RXPIN            PIND
# define TCCR             TCCR0A             // Timer/Counter Control Register
# define TCCR_P           TCCR0B             // Timer/Counter Control (Prescaler)
                                             // Register
# define OCR              OCR0A              // Output Compare Register
# define EXT_IFR          EIFR               // External Interrupt Flag Register
# define EXT_ICR          EICRA              // External Interrupt Control
                                             // Register
# define TIMER_COMP_VECT  TIMER0_COMPA_vect  // Timer Compare Interrupt Vector

#elif ( defined( __AVR_ATtiny24__ ) ) || defined( __AVR_ATtiny44__ ) || \
     defined( __AVR_ATtiny84__ )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK0 |= ( 1 << OCIE0A ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK0 &= ~( 1 << OCIE0A ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR0 |= ( ( 1 << OCF0A ) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   ( GIMSK |= ( 1 << INT0 ) )
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( GIMSK &= ~( 1 << INT0 ) )
# ifndef TX_PIN
#   define TX_PIN         PA0                // Transmit data pin
#   define TXDDR          DDRA
#   define TXPORT         PORTA
# endif
# define RX_PIN           PB2                // Receive data pin, must be INT0
# define RXDDR            DDRB
# define RXPORT           PORTB
# define TCCR             TCCR0A             // Timer/Counter Control Register
# define TCCR_P           TCCR0B             // Timer/Counter Control (Prescaler)
                                             // Register
# define OCR              OCR0A              // Output Compare Register
# define EXT_IFR          GIFR               // External Interrupt Flag Register
# define EXT_ICR          MCUCR              // External Interrupt Control
                                             // Register
# define TIMER_COMP_VECT  TIM0_COMPA_vect    // Timer Compare Interrupt Vector

#elif ( defined( __AVR_ATtiny25__ ) || defined( __AVR_ATtiny45__ ) || \
     defined( __AVR_ATtiny85__ ) )
# define ENABLE_TIMER_INTERRUPT( )       ( TIMSK |= ( 1 << OCIE0A ) )
# define DISABLE_TIMER_INTERRUPT( )      ( TIMSK &= ~( 1 << OCIE0A ) )
# define CLEAR_TIMER_INTERRUPT( )        ( TIFR |= ( ( 1 << OCF0A ) ) )
# define ENABLE_EXTERNAL0_INTERRUPT( )   { ACSR |= ( 1 << ACI ); ( ACSR |= (1<<ACIE) ); }
# define DISABLE_EXTERNAL0_INTERRUPT( )  ( ACSR &= ~(1<<ACIE) )
# ifndef TX_PIN
#   define TX_PIN         GPIO_LEDL                // Transmit data pin
#   define TXDDR          DDRB
#   define TXPORT         PORTB
# endif
# define RX_PIN           GPIO_LEDR                // Receive data pin, must be INT0
# define RXDDR            DDRB
# define RXPORT           PORTB
# define RXPIN            PINB
# define TCCR             TCCR0A             // Timer/Counter Control Register
# define TCCR_P           TCCR0B             // Timer/Counter Control (Prescaler)
                                             // Register
# define OCR              OCR0A              // Output Compare Register
                                             // Register
# define TIMER_COMP_VECT  TIMER0_COMPA_vect  // Timer Compare Interrupt Vector

#else
# error Selected AVR device is not supported

#endif

#define SET_TX_PIN( )    TXPORT |= ( 1 << TX_PIN );  RXPORT |= ( 1 << RX_PIN )
#define CLEAR_TX_PIN( )  TXPORT &= ~( 1 << TX_PIN ); RXPORT &= ~( 1 << RX_PIN )


#define GET_RX_PIN( )    ( !(ACSR   &   (1<<ACO)) )







static volatile AsynchronousStates_t state;     // Holds the state of the UART.

#ifndef RX_ONLY
static volatile unsigned char SwUartTXData;     // Data to be transmitted.
static volatile unsigned char SwUartTXBitCount; // TX bit counter.
#endif

#ifndef TX_ONLY
static volatile unsigned char SwUartRXData;     // Storage for received bits.
static volatile unsigned char SwUartRXData_Fifo;     // Storage for received bits.
static volatile unsigned char FifoHasByte = false;     // Storage for received bits.
static volatile unsigned char SwUartRXBitCount; // RX bit counter.
#endif



#ifndef TX_ONLY

/********************************************************************************

                      External interrupt service routine.

The falling edge in the beginning of the start bit will trig this interrupt.
The state will be changed to RECEIVE, and the timer interrupt will be set to
trig one and a half bit period from the falling edge.  At that instant the
code should sample the first data bit.

Note: initSoftwareUart( ) must be called in advance.

********************************************************************************/

ISR(ANA_COMP_vect)
{
		
  state = RECEIVE;                  // Change state
  DISABLE_EXTERNAL0_INTERRUPT( );   // Disable interrupt during the data bits.

  DISABLE_TIMER_INTERRUPT( );       // Disable timer to change its registers.
  TCCR_P &= ~CLOCK_SELECT_MASK;     // Reset prescaler counter.

                                    // Clear counter register. Include time to
                                    // run interrupt rutine to this point.
  TCNT0 = INTERRUPT_EXEC_CYCLES_COUNT;

  TCCR_P |= CLOCK_SELECT;           // Start prescaler clock.

  OCR = TICKS2WAITONE_HALF - 1;     // Count one and a half period into the
                                    // future.

  SwUartRXBitCount = 0;             // Clear received bit counter.
  FifoHasByte = false;
  CLEAR_TIMER_INTERRUPT( );         // Clear interrupt bits
  ENABLE_TIMER_INTERRUPT( );        // Enable timer0 interrupt on again
 

} // end ISR( INT0_vect )

#endif // ifndef TX_ONLY



/********************************************************************************

                       Timer0 interrupt service routine.

Timer0 will ensure that bits are written and read at the correct instants in
time.  The state variable will ensure context switching between transmit and
recieve.  If state should be something else, the variable is set to IDLE. IDLE is
regarded as a safe state/mode.

Note: initSoftwareUart( ) must be called in advance.

********************************************************************************/

ISR(
  TIMER_COMP_VECT
)
{

  switch ( state ) {

#ifndef RX_ONLY
  
    // Transmit Byte.
    case TRANSMIT:
      if( SwUartTXBitCount < 8 ) {            
        // Output the TX buffer.
        if( SwUartTXData & 0x01 ) {         // If the LSB of the TX buffer is 1:
          SET_TX_PIN( );                    // Send a logic 1 on the TX_PIN.
        }
        else
        {
          CLEAR_TX_PIN( );                  // Send a logic 0 on the TX_PIN.
        }
        SwUartTXData = SwUartTXData >> 1;   // Bitshift the TX buffer and
        SwUartTXBitCount++;                 // increment TX bit counter.
      }
      else
      {
        //Send stop bit.
        SET_TX_PIN( );                      // Output a logic 1.
        state = TRANSMIT_STOP_BIT;
      }
      break;

    // Go to idle after stop bit was sent.
    case TRANSMIT_STOP_BIT:
      DISABLE_TIMER_INTERRUPT( );           // Stop the timer interrupts.
      state = IDLE;                         // Go back to idle.
#ifndef TX_ONLY
      ACSR |= ( 1 << ACI );            // reset external interrupt 0
      ENABLE_EXTERNAL0_INTERRUPT( );        // Enable reception again.
#endif // ifndef TX_ONLY
      break;

#endif // ifndef RX_ONLY

#ifndef TX_ONLY

    // Receive Byte.
    case RECEIVE:
      OCR = TICKS2WAITONE - 1;              // Count one period after the falling
                                            // edge is trigged.
      // Receiving, LSB first.
      if( SwUartRXBitCount < 9 )
      {
        SwUartRXBitCount++;
        SwUartRXData = (SwUartRXData>>1);   // Shift due to receiving LSB first.
        if( GET_RX_PIN( ) != 0 )
        {
          SwUartRXData |= 0x80;             // If a logical 1 is read, let the
                                            // data mirror this.
        }
      }
      else
      {

        // Done receiving
        state = IDLE;               // Enter IDLE
		SwUartRXData_Fifo = SwUartRXData;
		FifoHasByte = true;
                                            // is received.
        DISABLE_TIMER_INTERRUPT( );         // Disable this interrupt.
        ACSR |= ( 1 << ACI );          // Reset flag not to enter the ISR
                                            // one extra time.
        ENABLE_EXTERNAL0_INTERRUPT( );      // Enable interrupt to receive more
                                            // bytes.
      }
      break;

#endif // ifndef TX_ONLY

    default:        
      // Unknown state.
      state = IDLE;                         // Error, should not occur. Going to
      DISABLE_TIMER_INTERRUPT();
	  ENABLE_EXTERNAL0_INTERRUPT();                                      // a safe state.

  } // end case

} // end ISR( TIMER_COMP_VECT )



/********************************************************************************

                                initSoftwareUart

********************************************************************************/

void
initSoftwareUart(
  void
)
{

  // PORT
  RXDDR &= ~( 1 << RX_PIN );        // RX_PIN is input, tri-stated.
  //RXPORT |= ( 1 << RX_PIN );
#ifndef RX_ONLY
  TXDDR |= ( 1 << TX_PIN );         // TX_PIN is output.
  SET_TX_PIN( );                    // Set the TX line to idle state.
#endif

  // Timer0
  DISABLE_TIMER_INTERRUPT( );
  TCCR = 0x00;                      // Init.
  TCCR_P = 0x00;                    // Init.
  TCCR |= ( 1 << WGM01 );           // Timer in CTC mode.
  TCCR_P |=  CLOCK_SELECT;          // set clock select

#ifndef TX_ONLY
  // External interrupt
	ADCSRB |= (1<<ACME);
	ACSR |= ( 1 << ACI );          // Reset flag not to enter the ISR
	ACSR |= (1<<ACBG);
	//trigger falling edge on rx pin
	ACSR |= (1<<ACIS1) | (0<<ACIS0);
	
	//ADMUX = ADCMUX_LEDR;
  ENABLE_EXTERNAL0_INTERRUPT( );    // Turn external interrupt on.
#endif

  //Internal State Variable
  state = IDLE;

} // end initSoftwareUart



#ifndef RX_ONLY

/********************************************************************************

                                    putChar

********************************************************************************/

int
putChar(char  c)
{

                                     // Don't send while busy receiving or
                                    // transmitting.
  while ( 
    ( state != IDLE )
#ifndef TX_ONLY
    &&
    ( state != DATA_PENDING )
#endif
  );

#ifndef TX_ONLY
  DISABLE_EXTERNAL0_INTERRUPT( );   // Disable reception.
#endif // ifndef TX_ONLY
  state = TRANSMIT;
  SwUartTXData = c;                 // Put byte into TX buffer.
  SwUartTXBitCount = 0;         

  TCCR_P &= ~CLOCK_SELECT_MASK;     // Reset prescaler counter.
  TCNT0 = 0;                        // Clear counter register.
  TCCR_P |= CLOCK_SELECT;           // CTC mode. Start prescaler clock.

  OCR = TICKS2WAITONE - 1;          // Set compare value to one bit period

  CLEAR_TIMER_INTERRUPT( );         // Clear interrupt bits

  CLEAR_TX_PIN( );                  // Clear TX line...start of preamble

  ENABLE_TIMER_INTERRUPT( );        // Enable interrupt

  return 0;

} // end putChar

#endif // ifndef RX_ONLY



#ifndef TX_ONLY

/********************************************************************************

                                    getChar

********************************************************************************/

int
getChar()
{

  while ( !FifoHasByte );
  FifoHasByte = false;

  return SwUartRXData_Fifo;

} // end getChar

#endif // ifndef TX_ONLY



#ifndef RX_ONLY

/********************************************************************************

				     txBusy

********************************************************************************/

bool
txBusy(
  void
)
{

  return ( state == TRANSMIT ) || ( state == TRANSMIT_STOP_BIT );

} // end if

#endif // ifndef RX_ONLY

void disbaleRXInt()
{
	state = IDLE;
	DISABLE_EXTERNAL0_INTERRUPT();
}

void enableRXInt()
{
	state = IDLE;
		
	ENABLE_EXTERNAL0_INTERRUPT();
}