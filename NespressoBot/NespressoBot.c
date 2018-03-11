/*
 * NespressoBot.c
 *
 * Created: 01.02.2016
 *  Author: Sebastian Foerster
 *  Webpage: sebastianfoerster86.wordpress.com
 */ 


#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include "NespressoBot.h"
#include "temperature/temperature.h"
#include "sound/songs.h"
#include "sound/sound.h"
#include "rand/parith-15.h"
#include "suart/swUart.h"

#define MAINLOOP_DELAY			10 //ms

////////////// UART PROTOCOL //////////////

#define PROTOCOL_START			0xEF
#define PROTOCOL_CMD_IAM		0xDF
#define PROTOCOL_END			0xFE

const uint8_t my_id __attribute__ ((section (".my_segment"))) = 0xFA;

uint8_t uart_msg[] = { PROTOCOL_START, PROTOCOL_CMD_IAM, 0, 0, PROTOCOL_END };
uint8_t uart_msg_cnt = 0;


///////////////////////////////////////////

uint8_t chr;

uint16_t measure_light(uint8_t gpio, uint8_t adc_mux);
void setup_adc();
uint8_t setSleep(void);
void long_delay(uint8_t delay);
void disable_analog();


uint8_t setSleep(void)
{
	//stop a possible playing song
	//stop interrupt executes so it will not set IOs
	playTune(0);
	
	//disable internal ref
	disable_analog();
	
	_delay_ms(10);
	
	//set all ports to input
	DDRB = 0;
	
	//enable pull up, set all other ports to zero
	PORTB = (1<<GPIO_SWITCH);
	
	//enable interrupt
	//MCUCR |= (0<<ISC01) | (0<<ISC00); //low level interrupt int0
	GIMSK |= (1<<INT0);
	GIFR |= (1<<INTF0);
	
	//go to sleep
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	sei();
	sleep_mode();
	
	////// after sleep /////////
	
	//disable interrupt
	GIMSK &=   ~(1<<INT0);
	
	//debounce wakeup pin
	for(uint8_t i = 0; i < 10; i++) {
		if((PINB & (1<<GPIO_SWITCH))) {
			return false;
		}
		_delay_ms(1);
	}
	
	//set outputs
	DDRB = (1<<GPIO_SPEAKER);
	
	return true;
}

// we need this vector to wakeup
ISR(INT0_vect)
{

}

void disable_analog()
{
	ADCSRA = 0;
	ADMUX = 0;
	ACSR = (1<<ACD);
}

void enableAnalogComp()
{
	//disable ADC
	ADCSRA = 0;
	
	ADMUX = ADCMUX_LEDR;
		
	ADCSRB |= (1<<ACME);
	
	ACSR &= ~(1<<ACD);
	//enable band gap ref 1.1 V
	ACSR |= (1<<ACBG);
	//trigger falling edge on rx pin
	ACSR |= (1<<ACIS1) | (1<<ACIS0);
		
}

uint16_t measure_light(uint8_t gpio, uint8_t adc_mux)
{
	//discharge LED
	DDRB |= (1<<gpio);
	DDRB &= ~(1<<gpio);
	
	//wait for new charge
	setup_adc();
	_delay_ms(20);
	
	//start adc measure
	ADMUX = adc_mux;
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) ;
	
	return ADCW;
}

void setup_adc()
{
	ACSR = (1<<ACBG); //disable analog comparator
	
	ADMUX = ADCMUX_LEDR;                 // select start channel (mux)
		
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2);// | (1<<ADIE); // frequency divider 64, ADC activate with interrupt
	
	//must be enabled to reduce the input current of the pins to use led as light sensor
	DIDR0 =  (1<<ADC2D) | (1<<ADC3D); //save power on adc input pin...
		
	ADCSRB = 0; //change conversation to free running;
}

void long_delay(uint8_t delay)
{
	do {
		_delay_ms(100);
	} while(--delay);
}

void display_temperature(void)
{
	setup_adc();
	
	int8_t temp10 = temperature();
	int8_t temp01 = temp10 % 10;
	temp10 /= 10;
	
	uint8_t a;
	
	DDRB |= ~(1<<GPIO_LEDR);
	PORTB &= ~(1<<GPIO_LEDR);
	DDRB |= ~(1<<GPIO_LEDL);
	PORTB &= ~(1<<GPIO_LEDL);
			
	//pattern to see that the temperature will be displayed!
	for(a = 0; a < 20; a++) 
	{
		PORTB ^= (1<<GPIO_LEDL) | (1<<GPIO_LEDR);
		long_delay(1);		
	}
	
	for(a = 0; a < 10; a++) 
	{
		
		PORTB &= ~(1<<GPIO_LEDR);
		PORTB &= ~(1<<GPIO_LEDL);
		
		long_delay(5);
		
		if(a < temp10)
		{
			PORTB |= (1<<GPIO_LEDR);
		}
		
		if(a < temp01)
		{
			PORTB |= (1<<GPIO_LEDL);
		}
		
		long_delay(5);
		
		//break if temperature display is done
		if(!(PORTB & ((1<<GPIO_LEDR) | (1<<GPIO_LEDL)))) 
		{
			break;
		}
	}
	
	DDRB &= ~(1<<GPIO_LEDR);
	PORTB &= ~(1<<GPIO_LEDR);
	DDRB &= ~(1<<GPIO_LEDL);
	PORTB &= ~(1<<GPIO_LEDL);
	
	long_delay(6);
	
}

uint8_t sleep_cnt = 0;
uint8_t song_reduce = 0;

uint8_t cnt_l = 0;
uint8_t cnt_r = 0;

//debugging
uint16_t	adc_data;
uint8_t		switch_debounce = 0;

//do we need this in the final application?
uint8_t saver;

int main( void )
{
	
	wdt_disable();
	
	//check if the temp is calibrated
	//erase eeprom if a wrong cal. is placed
	calibrate_temp(AIR_TEMPERATURE);
	
	//set outputs
	DDRB = (1<<GPIO_SPEAKER);
	
	//enable pull up from switch
	PORTB |= (1<<GPIO_SWITCH);
	
	//for a better randomization
	set_seed (measure_light(GPIO_LEDR, ADCMUX_LEDR));
	
	sei();
	
	//read ID to send string
	saver = pgm_read_byte(&my_id);
	uart_msg[2] = saver;
	uart_msg[3] = saver ^ 0xFF;
	
	saver = 0;
	
	//enable software uart transfer
	initSoftwareUart();
	disbaleRXInt();	
	
	//test sound here
	/*while(1) {
		for(uint8_t i = 1; i < NUMBER_OF_SONGS + 1; i++) {
			playTune(i);
			while(is_song_playing());
			long_delay(10);
		}
	}*/
	
	//display_temperature();
	
	while(1) {
		
		
		//if no automatic sleep is used -> the bot could be enabled and disabled by the keypad!
		/*
		//debounce key
		if(!(PINB & (1<<GPIO_SWITCH))) {
			switch_debounce++;
		} else {
			switch_debounce = 0;
		}
		//check key
		if(switch_debounce > 5)
		{
			PORTB &= ~(1<<GPIO_LEDR);
			PORTB &= ~(1<<GPIO_LEDL);
			//we are going to sleep
			for(int a = 0; a < 20; a++)
			{
				DDRB |= (1<<GPIO_LEDR) | (1<<GPIO_LEDL);
				PORTB ^= (1<<GPIO_LEDL) | (1<<GPIO_LEDR);
				_delay_ms(250);
			}
				
			while(!setSleep());
			//say hello
			DDRB |= (1<<GPIO_LEDR);
			PORTB |= (1<<GPIO_LEDR);
			DDRB |= (1<<GPIO_LEDL);
			PORTB |= (1<<GPIO_LEDL);
			long_delay(20); // wait 2 s
			//LEDs off
			PORTB |= (1<<GPIO_LEDR);
			PORTB |= (1<<GPIO_LEDL);
			
#ifndef BOT_GEBURTSTAG
			playTune(1);
#else
			playTune(4);
#endif
			long_delay(10); // wait 2 s
			//reset key
			switch_debounce = 0;
		}
		*/
		
		
		uint16_t max_t = prandom();
		
		if(!song_reduce) {
			
			//playing songs factor (reduce the overall playtime)
			song_reduce = 1;
			
#ifndef BOT_GEBURTSTAG
			if(max_t == 155) {
				playTune(1);
			}
		
			if(max_t == 25464) {
				playTune(2);
			} 
			
			if(max_t == 2764) {
				playTune(3);
			}
#else
			
			if(max_t == 155 || max_t == 25464 || max_t == 2764) {
				playTune(4);
			}
#endif
		
			//more random actions?
			/*
			if(max_t > 390 && max_t < 400) {
				//randomize the ocatave
				songlist[GET_DUR_INDEX(3)] = prandom();
				playTune(3);
			}
		
			if(max_t > 420 && max_t < 435) {
				//randomize the ocatave
				songlist[GET_DUR_INDEX(4)] = prandom();
				playTune(4);
			}
		
			if(max_t > 450 && max_t < 480) {
				//randomize the ocatave
				songlist[GET_DUR_INDEX(5)] = prandom();
				playTune(5);
			}
			
			if(max_t > 520 && max_t < 545) {
				display_temperature();				
			}*/
			
			if(max_t > 2500 && max_t < 2700) {
				DDRB |= (1<<GPIO_LEDR);
				PORTB |= (1<<GPIO_LEDR);
				DDRB |= (1<<GPIO_LEDL);
				PORTB |= (1<<GPIO_LEDL);
				
				// send uart "string" x times
				uart_msg_cnt = 0;
				for(adc_data = 0; adc_data < (3 * sizeof(uart_msg)); adc_data++) {
					//send ident msg and light up the leds
					if(uart_msg_cnt < sizeof(uart_msg)) {
						putChar(uart_msg[uart_msg_cnt]);
						uart_msg_cnt++;
					} 
					else 
					{
						uart_msg_cnt = 0;
					}
				}
				saver = 0;
				
				_delay_ms(10);
				
				PORTB &= ~(1<<GPIO_LEDR);
				PORTB &= ~(1<<GPIO_LEDL);
			}
		} else {
			song_reduce--;
		}		

		//right eye
		if(cnt_r == 0) {
			cnt_r = prandom();

			if(max_t & 0x01) {
				if((DDRB & (1<<GPIO_LEDR))) {
					saver = 1;
				}  else {
					//disbaleRXInt();
					DDRB |= (1<<GPIO_LEDR);
					PORTB |= (1<<GPIO_LEDR);
					cnt_r >>= 5;
				}
			} 
			else 
			{
				DDRB &= ~(1<<GPIO_LEDR);
				PORTB &= ~(1<<GPIO_LEDR);
			}
			
		} else {
			cnt_r--;
		}

		//left eye
		if(cnt_l == 0) {
			cnt_l = prandom();
			if(max_t & 0x02) {
				if(!(DDRB & (1<<GPIO_LEDL))) 
				{
					DDRB |= (1<<GPIO_LEDL);
					PORTB |= (1<<GPIO_LEDL);
					cnt_l >>= 5;
				}
			} 
			else 
			{
				PORTB &= ~(1<<GPIO_LEDL);
				DDRB &= ~(1<<GPIO_LEDL);
			}
		} else {
			cnt_l--;
		}
		

		//handle sleep routine
		if(!(DDRB & (1<<GPIO_LEDR))) {
			if(saver == 1 && !(DDRB & (1<<GPIO_LEDL))) {
				saver = 2;
				adc_data =  measure_light(GPIO_LEDR, ADCMUX_LEDR);
				
				//dark ?
				if(adc_data < 200) {
					if(sleep_cnt++ > 10) {
						while(!setSleep());
						sleep_cnt = 0;
						
						//LEDs on... say hello ;)
						DDRB |= (1<<GPIO_LEDR);
						PORTB |= (1<<GPIO_LEDR);
						DDRB |= (1<<GPIO_LEDL);
						PORTB |= (1<<GPIO_LEDL);
						long_delay(20); // wait 2 s
						//LEDs off
						PORTB |= (1<<GPIO_LEDR);
						PORTB |= (1<<GPIO_LEDL);
					}
				} else {
					sleep_cnt = 0;
				}
	
			} 		
		}
		
		_delay_ms(MAINLOOP_DELAY);
	}
}