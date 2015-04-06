#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

#include "servo.h"

#define MIN_ANGLE     40
#define MAX_ANGLE     140
#define ANGLE_OFFSET  -2

Servo::Servo()
	: m_CurAngle( 90 )
{
}

void
Servo::Enable()
{
	// Enable output to pin OC0A (PB7, "D11" on Romeo)
	PORTB &= ~( 1 << PORTB7 );
	DDRB |= ( 1 << DDB7 );

	// Configure and enable the PWM output to OC0 to about 90 degrees
	//
	// COM0A1:0 = 2 (OC0A high on OCR0A when down-counting)
	// COM0B1:0 = 0 (OC0B disconnected, normal port operation)
	// WGM02:0  = 1 (TOP = 0xFF)
	// CS02:0   = 4 (prescaler = 256)
	TCCR0A = ( 1 << COM0A1 ) | ( 1 << WGM00 );
	TCCR0B = ( 1 << CS02 );
	TCNT0 = 0xFF;
	SetAngle( m_CurAngle );
}

void
Servo::Disable()
{
	// Resets and stops the counter
	TCCR0A = 0;
	TCCR0B = 0;
}

void
Servo::SetAngle( uint8_t angle )
{
	angle += ANGLE_OFFSET;
	if( angle > 180 )
	{
		angle = 180;
	}
	m_CurAngle = angle;
	OCR0A = static_cast< uint8_t >( 16 + 62 * angle / 180 );
}