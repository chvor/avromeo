/*
 * AVR/Romeo experimental code.
 *
 * Used for a Romeo V2.2 (R3) board from www.dfrobot.com:
 * http://www.dfrobot.com/index.php?route=product/product&keyword=romeo%20v2&product_id=844
 * (the model might be obsolete/newer revision)
 *
 * It is using AVR ATmega32U4 with an Arduino firmware but I managed
 * to destroy my USB port and was never fond of the Arduino IDE anyway
 * so I'm using Atmel Studio (6.2 currently) and ISP to program it.
 *
 * JTAG works fine too but it disables 4 pins from port F (PF4..PF7, Arduino
 * analog ports A0, A1, A2 and A3).
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "HardwareSerial.h"

uint8_t
ADCsingleREAD( uint8_t adctouse )
{
	// left adjust for 8 bit, use AREF
	// MUX4..0 are in ADMUX; MUX5 is in ADCSRB
	ADMUX = ( adctouse & 0x1F ) | ( 1 << ADLAR ) | ( 1 << REFS0 );
	if( ( adctouse & 0x20 ) != 0 )
	{
		ADCSRB |= ( 1 << MUX5 );
	}
	else
	{
		ADCSRB &= ~( 1 << MUX5 );
	}

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
	ADCSRA |= (1 << ADEN);    // Enable the ADC

	ADCSRA |= (1 << ADSC);    // Start the ADC conversion

	while(ADCSRA & (1 << ADSC));      // Thanks T, this line waits for the ADC to finish

	return ADCH;
}

#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define NO 0

int
VoltageToKey( uint8_t adc )
{
	static const uint8_t val[] = { 20, 60, 100, 160, 200 };
	int i = 0;
	while( ( i < 5 ) && ( adc > val[i] ) )
	{
		++i;
	}
	if( i >= 5 )
	{
		return NO;
	}
	else
	{
		return i+1;
	}
}

void
blinkL( int times )
{
	for( int i = 0; i < times; ++i )
	{
		PORTC |= ( 1 << PORTC7 );
		_delay_ms( 50 );
		PORTC &= ~( 1 << PORTC7 );
		_delay_ms( 100 );
	}
}

void
toggleL()
{
	PINC |= ( 1 << PINC7 );
}

void
toggleTX()
{
	PIND |= ( 1 << PIND5 );
}

void
setupLeftMotor()
{
	// 8 bit Phase correct mode, counter/timer 3
	// compare unit A
	// Frequency: ~31 kHz
	// Usable values 155 <= N <= 255
	OCR3A = 0;
	TCCR3A = ( 1 << COM3A1 ) | ( 1 << WGM30 );
	TCCR3B = ( 1 << CS30 );
	
	// enable output to PD4 pin "D4"
	DDRD |= ( 1 << DDD4 );
	PORTD |= ( 1 << PORTD4 );

	// enable output to OC3A pin "D5"
	DDRC |= ( 1 << DDC6 );	
}

void
setupRightMotor()
{
	// phase and frequency correct mode, counter/timer 4
	// top set by OCR4C to 255, producing frequency ~31 kHz
	// (this could be set to produce exactly 25 kHz freq)
	// Usable values 155 <= N <= 255
	OCR4C = 255;
	OCR4D = 0;
	TCCR4C |= ( 1 << COM4D1 ) | ( 1 << PWM4D );
	TCCR4D |= ( 1 << WGM40 );
	TCCR4B |= ( 1 << CS40 );
	
	// enable output to PE6 pin "D7"
	DDRE |= ( 1 << DDE6 );
	PORTE |= ( 1 << PORTE6 );
	
	// enable output to OC4D/PD7 pin "D6"
	DDRD |= ( 1 << DDD7 );
}

void
leftMotorForward( uint16_t speed )
{
	PORTD |= ( 1 << PORTD4 );
	OCR3A = speed;
}

void
leftMotorBackward( uint16_t speed )
{
	PORTD &= ~( 1 << PORTD4 );
	OCR3A = speed;
}

void
leftMotorStop()
{
	OCR3A = 0;
}

void
rightMotorForward( uint16_t speed )
{
	PORTE |= ( 1 << PORTE6 );
	TC4H = ( speed >> 8 ) & 0x3;
	OCR4D = speed & 0xff;
}

void
rightMotorBackward( uint16_t speed )
{
	PORTE &= ~( 1 << PORTE6 );
	TC4H = ( speed >> 8 ) & 0x3;
	OCR4D = speed & 0xff;
}

void
rightMotorStop()
{
	TC4H = 0;
	OCR4D = 0;
}

void
fullStop()
{
	leftMotorStop();
	rightMotorStop();
}

#define AUTOMOVE_DELAY 1000

void
moveForward( uint8_t speed = 255 )
{
	leftMotorForward( speed );
	rightMotorForward( speed );
	_delay_ms( AUTOMOVE_DELAY );
	leftMotorStop();
	rightMotorStop();
}

void
moveBackward( uint8_t speed = 255 )
{
	leftMotorBackward( speed );
	rightMotorBackward( speed );
	_delay_ms( AUTOMOVE_DELAY );
	leftMotorStop();
	rightMotorStop();
}

void
turnLeft( uint8_t speed = 255 )
{
	leftMotorBackward( 255 );
	rightMotorForward( 255 );
	_delay_ms( AUTOMOVE_DELAY );
	leftMotorStop();
	rightMotorStop();
}

void
turnRight( uint8_t speed = 255 )
{
	leftMotorForward( speed );
	rightMotorBackward( speed );
	_delay_ms( AUTOMOVE_DELAY );
	leftMotorStop();
	rightMotorStop();
}

uint16_t
mapToPWM( int x)
{
	if( x < 10 )
	{
		return 0;
	}
	else if( x > 45 )
	{
		return 255;
	}
	return (uint16_t)( ( x - 10 ) * 3 + 150 );
}

void
motorControl( int l, int r )
{
	uint16_t p;
	if( l < 0 )
	{
		l = -l;
		PORTD &= ~( 1 << PORTD4 );
	}
	else
	{
		PORTD |= ( 1 << PORTD4 );
	}
	p = mapToPWM( l );
	OCR3A = p;
	if( r < 0 )
	{
		r = -r;
		PORTE &= ~( 1 << PORTE6 );
	}
	else
	{
		PORTE |= ( 1 << PORTE6 );
	}
	p = mapToPWM( r );
	TC4H = ( ( p >> 8 ) & 3 );
	OCR4D = (uint8_t)( p & 0xff );
}

// reads one integer number of up to X digits with 100us checks between checks for serial input
// X = sizeof buf below
// returns the first non-digit number
// accepts '+' and '-' and whitespace in the beginning
char
readIntWithTimeout( int & result, int & timeoutLeft, bool & valid )
{
	char r = 0;
	char buf[8];
	int pos = 0;
	bool gotSomething = false;
	valid = false;
	while( timeoutLeft > 0 && pos < (int)sizeof(buf) - 1 )
	{
		if( Serial1.available() )
		{
			buf[pos] = Serial1.read();
			if( ( !gotSomething && ( buf[pos] == '+' || buf[pos] == '-' ) )
				|| ( buf[pos] >= '0' && buf[pos] <= '9' ) )
			{
				gotSomething = true;
				++pos;
				buf[pos] = 0;
			}
			else if( !gotSomething && buf[pos] == ' ' ) // skip leading white space
			{
			}
			else
			{
				r = buf[pos];
				buf[pos] = 0; // erase latest char for conversion
				break;
			}
		}
		else
		{
			_delay_us( 100 );
		}
		--timeoutLeft;
	}
	if( ( pos == 1 ) && ( buf[0] == '+' || buf[0] == '-' ) )
	{
		pos = 0;
	}
	if( pos > 0 )
	{
		result = atoi(buf);
		valid = true;
	}
	return r;
}

int main(void)
{
	
	setupLeftMotor();
	setupRightMotor();
	sei();
	
	//uint16_t speed = 255;
	
	Serial1.begin(9600);
	
    while(1)
    {
#if 0
		uint8_t adc = ADCsingleREAD( 7 );
		if( adc < 240 )
		{
			uint8_t tmp = adc;
			do 
			{
				adc = tmp;
				tmp = ADCsingleREAD( 7 );
			} while ( tmp != adc );
			int but = VoltageToKey( adc );
			switch( but )
			{
			case S1: moveForward( speed ); break;
			case S3: fullStop(); break;
			case S2: moveForward( --speed ); break;
			case S4: moveForward( ++speed ); break;
			case S5: speed = 255; break;
			}
			while(1)
			{
				int tmp = ADCsingleREAD( 7 );
				if( tmp > 240 )
				{
					break;
				}
				_delay_ms( 10 );
			}
		}
#endif
		if( Serial1.available() )
		{
			char firstByte = Serial1.read();
			char tmp = firstByte;
			int l = 0, r = 0;
			int timeout = 10000;
			bool valid = false;
			
			if( firstByte == 'w' )
			{
				tmp = readIntWithTimeout( l, timeout, valid );
				if( valid && tmp != 10 )
				{
					tmp = readIntWithTimeout( r, timeout, valid );
				}				
			}
			else if( firstByte == 'v' )
			{
				tmp = readIntWithTimeout( r, timeout, valid );
			}
			
			while( tmp != 10 && timeout > 0 )
			{
				--timeout;
				if( Serial1.available() )
				{
					tmp = Serial1.read();
				} else
				{
					_delay_us( 100 );
				}
			}
			
			Serial1.write( firstByte );
			Serial1.write( 10 );

			switch( firstByte )
			{
			case 's':
				fullStop();
				break;
				
			case 'f':
				moveForward();
				break;
				
			case 'b':
				moveBackward();
				break;
				
			case 'l':
				turnLeft();
				break;
				
			case 'r':
				turnRight();
				break;
			
			case 'w':
				motorControl( l, r );
				break;
				
			case 'v':
				break;
				
			default:
				break;
			}
			
			Serial1.write( '$' );
			Serial1.write( 10 );
		}
    }
}
