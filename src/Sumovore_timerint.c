//	 	Sumovore_timerint.c
//
// written by Justin Lee, Smart Devices Lab, QUT Nov. 2005
//
// Outputs a log2 representation of the number of ticks modulus 512,
// using the red LEDs for the low 8 bits, and the green LED for the 9th bit.
// Demonstrates the use of Timer interrupts, for generating a ~ 10ms tick
// counter with timer 2 
//
// For the Sumovore mobile robot with the Atmel ATmega8(L) Brainboard add-on
// Compiled using GNU GCC with avr-libc 1.2.3 (from WinAVR-20050214)
//
// Note: code has been written assuming a 1 MHz MCU CLK, if this is not the
// case, then CLK prescalers will need to be modified to suit actual frequency

/*

   Sumovore pinout for the ATmega8(L) Microcontroller in a PDIP package

               +---------------+
         reset |  1     PC5 28 | N.C.
      IR left  |  2 PD0 PC4 27 | Edge sensor right        (ADC4)
      IR right |  3 PD1 PC3 26 | Edge sensor center right (ADC3)
     (L) LED 1 |  4 PD2 PC2 25 | Edge sensor center       (ADC2)
         LED 2 |  5 PD3 PC1 24 | Edge sensor center left  (ADC1)
     (C) LED 3 |  6 PD4 PC0 23 | Edge sensor left         (ADC0)
           VCC |  7         22 | GND
           GND |  8         21 | AREF
        xtal 1 |  9 PB6     20 | AVCC
        xtal 2 | 10 PB7 PB5 19 | right motor direction
         LED 4 | 11 PD5 PB4 18 | left motor direction
     (R) LED 5 | 12 PD6 PB3 17 | Servo 1 / green LED
       Servo 2 | 13 PD7 PB2 16 | right motor enable/PWM   (OC1B)
       Servo 3 | 14 PB0 PB1 15 | left motor enable/PWM    (OC1A)
               +---------------+

 */



// for 8- and 16-bit unsigned int types (uint8_t uint16_t)
#include <stdint.h>

// (IO port) register definitions, IO and interrupt signal bit definitions
#include <avr/io.h>

// Interrupt registers and bit definitions, sei() and cli() macros
#include <avr/interrupt.h>

// Macros for writing interrupt handler functions
#include <avr/signal.h>


// define CLK frequency as 1 MHz
#define MCU_FREQ		1000000UL


// define mnemonics for IO ports
// note that IR detectors are inputs so use PIND (in) not PORTD (out)
#define MOTORPORT	PORTB
#define GREENPORT	PORTB
#define REDPORT		PORTD
#define IRPIN		PIND


// define mnemonics for IO pins
// note Left to Right defined looking from the back of the sumovore
#define GREENLED	PB3
#define REDLED_L	PD2
#define REDLED_ML	PD3
#define REDLED_C	PD4
#define REDLED_MR	PD5
#define REDLED_R	PD6
#define LMOTOR_EN	PB1
#define RMOTOR_EN	PB2
#define LMOTOR_FORWARD	PB4
#define RMOTOR_FORWARD	PB5

// define mnemonics for the ADC channels corresponding to the edge sensors
// (L to R from back of sumovore)
#define LINESENSOR_L	0
#define LINESENSOR_ML	1
#define LINESENSOR_C	2
#define LINESENSOR_MR	3
#define LINESENSOR_R	4

// define mnemonics for IR detectors at front of sumovore
#define IR_L	PD0
#define IR_R	PD1

// define mnemonics for PWM duty cycle registers
#define LMOTOR_DUTY	OCR1A
#define RMOTOR_DUTY	OCR1B




// function definitions
uint8_t getlevel(uint8_t value);
void initialise(void);
void outlevel(uint8_t level);
void redsoff(void);
void setup10msTimer(void);
void setupIO(void);



// Timer/Counter2 Compare Match interrupt handler definition
// Warning: use signal names from avr/iom8.h NOT the datasheet (p. 44)!
INTERRUPT(SIG_OUTPUT_COMPARE2);



// define the 10ms resolution tick counter that is updated by the
// timer interrupt handler. the "volatile" keyword tells the compiler
// that tickcnt can change unexpectedly (i.e. during an interrupt).
volatile uint16_t tickcnt;


int main(void)
{

  cli();				// turn interrupts off for setup
  setupIO();				// setup IO ports

  // not using the motors, so turn them off via left and right motor enable
  MOTORPORT &= ~((1 << LMOTOR_EN) | (1 << RMOTOR_EN));

  setup10msTimer();			// start up timer & enable interrupts
  // sei();				// re-enable interrupts

  while(1) {
    outlevel(getlevel(tickcnt));	// 8-bit logarithmic count
    if ((tickcnt/0x100)%2) {		// 9th bit
      PORTB |= (1 << GREENLED);
    } else {
      PORTB &= ~(1 << GREENLED);
    }
  }


  // never reach here, it's just to keep compiler happy
  return 0;
}



// to use IO pins on the AVR need to tell it which are inputs and which
// are outputs. this is done by setting the appropriate bits in the
// data direction registers (DDRx) to 0's for inputs and 1's for outputs.
// the LEDs and motors are outputs, so their DDRx bits are specifically set,
// while the ADC channel input pins are implicitly set to 0's
void setupIO()
{
  // red LEDs are on port D
  DDRD = (1 << REDLED_L) | (1 << REDLED_ML)
       | (1 << REDLED_C) | (1 << REDLED_MR)
       | (1 << REDLED_R);

  // green LED, left and right motor enable and direction are on port B
  DDRB = (1 << GREENLED)
       | (1 << LMOTOR_EN ) | (1 << RMOTOR_EN)
       | (1 << LMOTOR_FORWARD) | (1 << RMOTOR_FORWARD);
}



// this sets up 8-bit Timer/Counter 2 to tick over once every 10ms,
// generating an interrupt that increments a global 10ms counter. 
// Note: with a 1MHz CPU clk, and 256 prescalar, 10ms = 39.0625 ticks
// (1sec=3906.25), so if round to 39 ticks, then this gives an error of
// -0.16%, or 1 sec per 10 min 25 sec
// To do this use Clear Timer on Compare Match (CTC) Mode (WGM21:20 = 2),
// in which the counter is cleared to zero when the counter value (TCNT2)
// matches the OCR2 register. OCR2 defines the top value for the counter,
// and so also its resolution.
void setup10msTimer()
{
  // disable interrupts (in case it hasn't already been done)
  cli();

  // set Clear Timer on Compare Match (CTC) mode (TOP = OCR2)
  TCCR2 = (1 << WGM21);

  // set timer clk = CLK/256 -> 1Mhz / 256
  TCCR2 |= (1 << CS22) | (1 << CS21);

  // set timer TOP to 39. ie interrupt every 39 timer cycles ~ 10ms
  OCR2 = 39;

  // enable timer interrupt only on Output Compare Flag 2 (TCNT2 = OCR2)
  TIMSK = (1 << OCIE2);

  // and initialise tickcnt to zero
  tickcnt=0;

  // re-enable interrupts
  sei();
}



// Timer/Counter2 Compare Match interrupt handler
// note that tickcnt is uint16_t, so with an interrupt every 10ms,
// it can count for around 10:55sec before resetting
// Note use of INTERRUPT() vs SIGNAL(), this is as don't care if get preempted
// as this is not critical (and 10ms is plenty of time to handle it)

INTERRUPT(SIG_OUTPUT_COMPARE2)
{
  ++tickcnt;				// note ignoring overflow
}




// convert a value to a level of magnitude
uint8_t getlevel(uint8_t value)
{
  uint8_t level=0;
  while (value) {
    level++;
    value >>= 1;
  }

  return level;
}



// uses the 5 red LEDs to output a sensor's level from 0-8
// off=0, left -> right = level 1-5, right -> left = level 6-8
void outlevel(uint8_t level)
{
  redsoff();			// start with all off = 0
  if (level<6) {		// light from left to right for levels 1-5
    while (level--) {
      PORTD |= (1 << (level+2)); // cheat! as avr/iom8.h defines PDx as x
    }
  } else {			// >5 so light from right to left for 6-8
    PORTD |= (1 << REDLED_R);	// level 6
    if (level > 6) {
      PORTD |= (1 << REDLED_MR); // level 7
    }
    if (level > 7) {
      PORTD |= (1 << REDLED_C); // level 8
    }
  }
}



// turn off all the red leds
void redsoff()
{
  PORTD &= ~( (1 << REDLED_L) | (1 << REDLED_ML)
            | (1 << REDLED_C) | (1 << REDLED_MR)
            | (1 << REDLED_R));
}

