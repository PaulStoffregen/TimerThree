/*
 *  Interrupt and PWM utilities for 16 bit Timer3 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified April 2012 by Paul Stoffregen - portable to other AVR chips, use inline functions
 *  Modified again, June 2014 by Paul Stoffregen - support Teensy 3.1 & even more AVR chips
 *
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


#if defined(__AVR__)
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
	resume();
    }
    void stop() __attribute__((always_inline)) {
	TCCR3B = _BV(WGM33);
    }
    void restart() __attribute__((always_inline)) {
	start();
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
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;



#elif defined(__arm__) && defined(TEENSYDUINO) && (defined(KINETISK) || defined(KINETISL))

#if defined(KINETISK)
#define F_TIMER F_BUS
#elif defined(KINETISL)
#define F_TIMER (F_PLL/2)
#endif

// Use only 15 bit resolution.  From K66 reference manual, 45.5.7 page 1200:
//   The CPWM pulse width (duty cycle) is determined by 2 x (CnV - CNTIN) and the
//   period is determined by 2 x (MOD - CNTIN). See the following figure. MOD must be
//   kept in the range of 0x0001 to 0x7FFF because values outside this range can produce
//   ambiguous results.
#undef TIMER3_RESOLUTION
#define TIMER3_RESOLUTION 32768

  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	const unsigned long cycles = (F_TIMER / 2000000) * microseconds;

  /*
  // This code does not work properly in all cases :(
  // https://github.com/PaulStoffregen/TimerOne/issues/17 
  if (cycles < TIMER3_RESOLUTION * 16) {
    if (cycles < TIMER3_RESOLUTION * 4) {
      if (cycles < TIMER3_RESOLUTION) {
        clockSelectBits = 0;
        pwmPeriod = cycles;
      }else{
        clockSelectBits = 1;
        pwmPeriod = cycles >> 1;
      }
    }else{
      if (cycles < TIMER3_RESOLUTION * 8) {
        clockSelectBits = 3;
        pwmPeriod = cycles >> 3;
      }else{
        clockSelectBits = 4;
        pwmPeriod = cycles >> 4;
      }
    }
  }else{
    if (cycles > TIMER3_RESOLUTION * 64) {
      if (cycles > TIMER3_RESOLUTION * 128) {
        clockSelectBits = 7;
        pwmPeriod = TIMER3_RESOLUTION - 1;
      }else{
        clockSelectBits = 7;
        pwmPeriod = cycles >> 7;
      }
    }else{
      if (cycles > TIMER3_RESOLUTION * 32) {
        clockSelectBits = 6;
        pwmPeriod = cycles >> 6;
      }else{
        clockSelectBits = 5;
        pwmPeriod = cycles >> 5;
      }
    }
  }
  */
	if (cycles < TIMER3_RESOLUTION) {
		clockSelectBits = 0;
		pwmPeriod = cycles;
	} else
	if (cycles < TIMER3_RESOLUTION * 2) {
		clockSelectBits = 1;
		pwmPeriod = cycles >> 1;
	} else
	if (cycles < TIMER3_RESOLUTION * 4) {
		clockSelectBits = 2;
		pwmPeriod = cycles >> 2;
	} else
	if (cycles < TIMER3_RESOLUTION * 8) {
		clockSelectBits = 3;
		pwmPeriod = cycles >> 3;
	} else
	if (cycles < TIMER3_RESOLUTION * 16) {
		clockSelectBits = 4;
		pwmPeriod = cycles >> 4;
	} else
	if (cycles < TIMER3_RESOLUTION * 32) {
		clockSelectBits = 5;
		pwmPeriod = cycles >> 5;
	} else
	if (cycles < TIMER3_RESOLUTION * 64) {
		clockSelectBits = 6;
		pwmPeriod = cycles >> 6;
	} else
	if (cycles < TIMER3_RESOLUTION * 128) {
		clockSelectBits = 7;
		pwmPeriod = cycles >> 7;
	} else {
		clockSelectBits = 7;
		pwmPeriod = TIMER3_RESOLUTION - 1;
	}

	uint32_t sc = FTM2_SC;
	FTM2_SC = 0;
	FTM2_MOD = pwmPeriod;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_CPWMS | clockSelectBits | (sc & FTM_SC_TOIE);
    }

    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	stop();
	FTM2_CNT = 0;
	resume();
    }
    void stop() __attribute__((always_inline)) {
	FTM2_SC = FTM2_SC & (FTM_SC_TOIE | FTM_SC_CPWMS | FTM_SC_PS(7));
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	FTM2_SC = (FTM2_SC & (FTM_SC_TOIE | FTM_SC_PS(7))) | FTM_SC_CPWMS | FTM_SC_CLKS(1);
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	unsigned long dutyCycle = pwmPeriod;
	dutyCycle *= duty;
	dutyCycle >>= 10;
	if (pin == TIMER3_A_PIN) {
		FTM2_C0V = dutyCycle;
	} else if (pin == TIMER3_B_PIN) {
		FTM2_C1V = dutyCycle;
	}
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	setPwmDuty(pin, duty);
	if (pin == TIMER3_A_PIN) {
		*portConfigRegister(TIMER3_A_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
	} else if (pin == TIMER3_B_PIN) {
		*portConfigRegister(TIMER3_B_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER3_A_PIN) {
		*portConfigRegister(TIMER3_A_PIN) = 0;
	} else if (pin == TIMER3_B_PIN) {
		*portConfigRegister(TIMER3_B_PIN) = 0;
	}
    }

    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	FTM2_SC |= FTM_SC_TOIE;
	NVIC_ENABLE_IRQ(IRQ_FTM2);
    }
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	FTM2_SC &= ~FTM_SC_TOIE;
	NVIC_DISABLE_IRQ(IRQ_FTM2);
    }
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;

#undef F_TIMER

#elif defined(__arm__) && defined(TEENSYDUINO) && defined(__IMXRT1062__)

  public:
    //****************************
    //  Configuration
    //****************************
    void initialize(unsigned long microseconds=1000000) __attribute__((always_inline)) {
	setPeriod(microseconds);
    }
    void setPeriod(unsigned long microseconds) __attribute__((always_inline)) {
	uint32_t period = (float)F_BUS_ACTUAL * (float)microseconds * 0.0000005f;
	uint32_t prescale = 0;
	while (period > 32767) {
		period = period >> 1;
		if (++prescale > 7) {
			prescale = 7;	// when F_BUS is 150 MHz, longest
			period = 32767; // period is 55922 us (~17.9 Hz)
			break;
		}
	}
	//Serial.printf("setPeriod, period=%u, prescale=%u\n", period, prescale);
	FLEXPWM2_FCTRL0 |= FLEXPWM_FCTRL0_FLVL(4); // logic high = fault
	FLEXPWM2_FSTS0 = 0x0008; // clear fault status
	FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(4);
	FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP;
	FLEXPWM2_SM2CTRL = FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale);
	FLEXPWM2_SM2INIT = -period;
	FLEXPWM2_SM2VAL0 = 0;
	FLEXPWM2_SM2VAL1 = period;
	FLEXPWM2_SM2VAL2 = 0;
	FLEXPWM2_SM2VAL3 = 0;
	FLEXPWM2_SM2VAL4 = 0;
	FLEXPWM2_SM2VAL5 = 0;
	FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(4) | FLEXPWM_MCTRL_RUN(4);
	pwmPeriod = period;
    }
    //****************************
    //  Run Control
    //****************************
    void start() __attribute__((always_inline)) {
	stop();
	// TODO: how to force counter back to zero?
	resume();
    }
    void stop() __attribute__((always_inline)) {
	FLEXPWM2_MCTRL &= ~FLEXPWM_MCTRL_RUN(4);
    }
    void restart() __attribute__((always_inline)) {
	start();
    }
    void resume() __attribute__((always_inline)) {
	FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_RUN(4);
    }

    //****************************
    //  PWM outputs
    //****************************
    void setPwmDuty(char pin, unsigned int duty) __attribute__((always_inline)) {
	if (duty > 1023) duty = 1023;
	int dutyCycle = (pwmPeriod * duty) >> 10;
	//Serial.printf("setPwmDuty, period=%u\n", dutyCycle);
	if (pin == TIMER3_A_PIN) {
		FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(4);
		FLEXPWM2_SM2VAL5 = dutyCycle;
		FLEXPWM2_SM2VAL4 = -dutyCycle;
		FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(4);
	} else if (pin == TIMER3_B_PIN) {
		FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(4);
		FLEXPWM2_SM2VAL3 = dutyCycle;
		FLEXPWM2_SM2VAL2 = -dutyCycle;
		FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(4);
	}
    }
    void pwm(char pin, unsigned int duty) __attribute__((always_inline)) {
	setPwmDuty(pin, duty);
	if (pin == TIMER3_A_PIN) {
		FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMB_EN(4);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 2; // pin 9 FLEXPWM2_PWM2_B
	} else if (pin == TIMER3_B_PIN) {
		FLEXPWM2_OUTEN |= FLEXPWM_OUTEN_PWMA_EN(4);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 2; // pin 6 FLEXPWM2_PWM2_A
	}
    }
    void pwm(char pin, unsigned int duty, unsigned long microseconds) __attribute__((always_inline)) {
	if (microseconds > 0) setPeriod(microseconds);
	pwm(pin, duty);
    }
    void disablePwm(char pin) __attribute__((always_inline)) {
	if (pin == TIMER3_A_PIN) {
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 5; // pin 9 FLEXPWM2_PWM2_B
		FLEXPWM2_OUTEN &= ~FLEXPWM_OUTEN_PWMB_EN(4);
	} else if (pin == TIMER3_B_PIN) {
		IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 5; // pin 6 FLEXPWM2_PWM2_A
		FLEXPWM2_OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(4);
	}
    }
    //****************************
    //  Interrupt Function
    //****************************
    void attachInterrupt(void (*f)()) __attribute__((always_inline)) {
	isrCallback = f;
	attachInterruptVector(IRQ_FLEXPWM2_2, &isr);
	FLEXPWM2_SM2STS = FLEXPWM_SMSTS_RF;
	FLEXPWM2_SM2INTEN = FLEXPWM_SMINTEN_RIE;
	NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_2);
    }
    void attachInterrupt(void (*f)(), unsigned long microseconds) __attribute__((always_inline)) {
	if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(f);
    }
    void detachInterrupt() __attribute__((always_inline)) {
	NVIC_DISABLE_IRQ(IRQ_FLEXPWM2_2);
	FLEXPWM2_SM2INTEN = 0;
    }
    static void isr(void);
    static void (*isrCallback)();
    static void isrDefaultUnused();

  private:
    // properties
    static unsigned short pwmPeriod;
    static unsigned char clockSelectBits;


#endif
};

extern TimerThree Timer3;

#endif

