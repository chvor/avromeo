#include <avr/io.h>
#include "dist.h"

void
Dist::Enable()
{
	// PB4, "D8" -> COMP/TRIG on URM37 (distance sensor)
	DDRB |= ( 1 << DDB4 );
	PORTB |= ( 1 << PORTB4 );

	// disable output to OC4A/PC7 pin "D13" also attached to led "L"
	// this is sourced by the URM37 sensor's "PWM" pin (led "L" is always ON)
	// when a PWM measurement is sent it is pulled down (led "L" goes OFF)
	DDRC &= ~( 1 << DDC7 );
}
