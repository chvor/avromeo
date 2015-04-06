#ifndef SERVO_H_
#define SERVO_H_

/*
 * Written for a standard 5V servo, more specifically the DF05BB model from dfrobot.com:
 * http://www.dfrobot.com/index.php?route=product/product&keyword=df05&product_id=236
 * It is controlled digitally via periodic pulses between 500 and 2500 microseconds.
 * The neutral (center) position is selected via a 1500us pulse.
 * The frequency of the pulses can vary between 50 and 330 Hz.
 *
 * When using the 8-bit Timer/Counter 0 in Phase Correct PWM Mode a useful prescaler
 * setting is 256. This means, for 16 MHz main clock, 16 MHz / ( 510 * 256 ) =~ 122 Hz,
 * or exactly 8160 microseconds per cycle. (ATmega32U4 data sheet, page 92-93, #13.6.4)
 *
 * Setting WGM2:0 to 1 puts the TOP to MAX, meaning the timer always counts from 0 to
 * MAX and then back to 0. For the 8 bit timer/counter MAX equals 0xFF (255) and because
 * the values 0 and 255 are hit only once the cycle has 255 * 2 = 510 values.
 *
 * Setting COM0A1:0 to 2 selects normal compare match mode for the OC0A pin. It will
 * be set when the counter is equal to OCR0A when down-counting and cleared when the
 * counter is equal to OCR0A again when up-counting. The changes done to OCR0A are only
 * applied when the counter reaches TOP. This will mean the high pulses sent over the
 * OC0A pin will always be symmetric.
 *
 * Following the above timer/counter description, the following formula for OCR0A can
 * be used for a prescaler of 256, allowing for a 122 Hz update of the servo:
 *
 *   OCR0A = X * 255 / 8160,
 *
 * This has a minimum of 0us, maximum of 8160us and a step of 32us. The minimum useful
 * value is 512us (OCR0A=16), the maximum is 2496us (OCR0A=78) and for a 180 degree
 * servo this means a step of about 2.857 degrees. For a small servo this is quite
 * good actually.
 *
 * The formula can be converted for degrees (R, 0..180):
 *
 *   OCR0A = 16 + 62 * R / 180
 *
 * Due to the hardware arrangement on my own robot there is an additional limit on the
 * servo rotation. It is hard-coded at compile time for now (MIN_ANGLE and MAX_ANGLE in
 * the servo.cpp file). Another hardware arrangement dictates the center position to
 * be 87 degrees instead of 90. This is set in ANGLE_OFFSET.
 */

class Servo
{
public:
	Servo();
	
	void Enable();
	void Disable();
	void SetAngle( uint8_t angle = 90 );
	uint8_t GetAngle() const
	{
		return m_CurAngle;
	}

protected:
	uint8_t m_CurAngle;
};

#endif /* SERVO_H_ */
