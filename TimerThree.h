/*
 *  Interrupt and PWM utilities for 16 bit Timer3 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified April 2012 by Paul Stoffregen - portable to other AVR chips, use inline functions
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#ifndef TimerThree_h_
#define TimerThree_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "config/known_16bit_timers.h"

#define TIMER3_RESOLUTION 65536UL  // Timer3 is 16 bit


// Placing nearly all the code in this .h file allows the functions to be
// inlined by the compiler.  In the very common case with constant values
// the compiler will perform all calculations and simply write constants
// to the hardware registers (for example, setPeriod).


class TimerThree
{
  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	TCCR3B = _BV(WGM33);        // set mode as phase and frequency correct pwm, stop the timer
	TCCR3A = 0;                 // clear control register A 
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	const unsigned long cycles = (F_CPU / 2000000) * microseconds;
	if (cycles < TIMER3_RESOLUTION) {
		clockSelectBits = _BV(CS30);
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER3_RESOLUTION * 8) {
		clockSelectBits = _BV(CS31);
		pwmPeriod = cycles / 8;
	} else
	if (cycles < TIMER3_RESOLUTION * 64) {
		clockSelectBits = _BV(CS31) | _BV(CS30);
		pwmPeriod = cycles / 64;
	} else
	if (cycles < TIMER3_RESOLUTION * 256) {
		clockSelectBits = _BV(CS32);
		pwmPeriod = cycles / 256;
	} else
	if (cycles < TIMER3_RESOLUTION * 1024) {
		clockSelectBits = _BV(CS32) | _BV(CS30);
		pwmPeriod = cycles / 1024;
	} else {
		clockSelectBits = _BV(CS32) | _BV(CS30);
		pwmPeriod = TIMER3_RESOLUTION - 1;
	}
	ICR3 = pwmPeriod;
	TCCR3B = _BV(WGM33) | clockSelectBits;
    }

    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	TCCR3B = 0;
	TCNT3 = 0;		// TODO: does this cause an undesired interrupt?
	TCCR3B = _BV(WGM33) | clockSelectBits;
    }
    void stop() __attribute__((always_inline)) {
	TCCR3B = _BV(WGM33);
    }
    void restart() __attribute__((always_inline)) {
	TCCR3B = 0;
	TCNT3 = 0;
	TCCR3B = _BV(WGM33) | clockSelectBits;
    }
    void resume() __attribute__((always_inline)) {
	TCCR3B = _BV(WGM33) | clockSelectBits;
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	unsigned long dutyCycle = pwmPeriod;
	dutyCycle *= duty;
	dutyCycle >>= 10;
	if (pin == TIMER3_A_PIN) OCR3A = dutyCycle;
	#ifdef TIMER3_B_PIN
	else if (pin == TIMER3_B_PIN) OCR3B = dutyCycle;
	#endif
	#ifdef TIMER3_C_PIN
	else if (pin == TIMER3_C_PIN) OCR3C = dutyCycle;
	#endif
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	if (pin == TIMER3_A_PIN) { pinMode(TIMER3_A_PIN, OUTPUT); TCCR3A |= _BV(COM3A1); }
	#ifdef TIMER3_B_PIN
	else if (pin == TIMER3_B_PIN) { pinMode(TIMER3_B_PIN, OUTPUT); TCCR3A |= _BV(COM3B1); }
	#endif
	#ifdef TIMER3_C_PIN
	else if (pin == TIMER3_C_PIN) { pinMode(TIMER3_C_PIN, OUTPUT); TCCR3A |= _BV(COM3C1); }
	#endif
	setPwmDuty(pin, duty);
	TCCR3B = _BV(WGM33) | clockSelectBits;
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER3_A_PIN) TCCR3A &= ~_BV(COM3A1);
	#ifdef TIMER3_B_PIN
	else if (pin == TIMER3_B_PIN) TCCR3A &= ~_BV(COM3B1);
	#endif
	#ifdef TIMER3_C_PIN
	else if (pin == TIMER3_C_PIN) TCCR3A &= ~_BV(COM3C1);
	#endif
    }

    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	TIMSK3 = _BV(TOIE3);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	TIMSK3 = 0;
    }
    void (*isrCallback)();

  protected:
    // properties
    static unsigned int pwmPeriod;
    static unsigned char clockSelectBits;
};

extern TimerThree Timer3;

#endif

