/* avr8 temperature reading library
 * http://nerdralph.googlecode.com
 * @author: Ralph Doncaster 2014
 */
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#define ADCINPUT (0<<REFS0) | (1<<REFS1) | (0x0F)

#define ADC_GAIN 1.06
#define SAMPLE_COUNT ((256/ADC_GAIN)+0.5)

uint8_t temp_offset EEMEM;

static uint16_t doADC(void)
{
	// start conversion with ADC clock prescaler 16
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADPS2);
	while (ADCSRA & (1<<ADSC));         // wait to finish
	return ADCW;
}

// returns signed byte for temperature in oC
int8_t temperature(void)
{
	ADMUX = ADCINPUT;
	uint16_t tempRaw = 0;
	// take multiple samples then average
	for (uint8_t count = SAMPLE_COUNT; --count;) {
		tempRaw += (doADC() - 273);
	}

	// a known offset could be used instead of the calibrated value
	return ((tempRaw/256) - eeprom_read_byte(&temp_offset) ) ;
}

void calibrate_temp(uint8_t act_temp)
{
	if ( eeprom_read_byte(&temp_offset) == 0xff)
	{
		// temperature uncalibrated
		char tempVal = temperature();   // throw away 1st sample
		tempVal = temperature();
		// 0xff == -1 so final offset is reading - AIR_TEMPERATURE -1
		eeprom_write_byte( &temp_offset, (tempVal - act_temp) -1);
	}
}