//		Sumovore_lineSensorMeter.c
//
// written by Justin Lee, Smart Devices Lab, QUT Nov. 2005
// 
// Tests the line sensors, and displays either the magnitude or 8-bit sensor
// values, with mode chosen according to whether the IR detectors are
// presented with an obstacle on startup. This demonstrates the use of ADC,
// IR detectors and the green and red LEDs.
//
// If presented with obstacle on startup Sumovore used as:
// 	- a reflectivity level meter for measuring the response of the chosen
// 	  line sensor to different backgrounds, black lines, etc
// 	  sensor chosen by reading from IR detectors: if obstacle detected on
// 	  L or R or both, then L, R, C sensor chosen, repsectively
// otherwise Sumovore used as:
//	- measure the response of all the line sensors, displaying the full
//	  8-bit values for each (0-225), with lower values corresponding to
//	  lighter (reflective) surfaces, while higher values correspond to
//	  darker (absorbing) surfaces
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

void countdown(void);
void displayvalue(uint8_t value, uint8_t pause);
void flashgreen(uint8_t secs);
void flashred(uint8_t ledno, uint8_t secs);
uint8_t getlevel(uint8_t value);
void levelmeter(uint8_t channel);
void lineResponseMeter(void);
void outlevel(uint8_t level);
uint8_t readADC(uint8_t channel);
void redsoff(void);
void setupADC(void);
void setupIO(void);
void testIRdetectors(void);
void waitsecs(uint8_t secs);
void waitMS(uint16_t msec);
void waitUS(uint16_t usec);



int main(void)
{
  cli();				// turn interrupts off for setup
  setupIO();				// setup IO ports

  // not using the motors, so turn them off via left and right motor enable
  MOTORPORT &= ~((1 << LMOTOR_EN) | (1 << RMOTOR_EN));

  flashgreen(2);			// flash green to show IO setup OK
  setupADC();				// setup ADC for line sensors
  countdown();				// 5 sec countdown ...

  // uncomment to just test the IR detectors
  //testIRdetectors();			// indicate what IRs are detecting

  // if detect an obstacle (ie hand) then use that to indicate which
  // line sensor to read from for the signal magnitude meter
  // note sensor returns 0 on detection, 1 on not detected
  if (!(IRPIN & (1 << IR_L)) && !(IRPIN & (1 << IR_R))) {
    levelmeter(LINESENSOR_C);		// magnitude meter for C line sensor
  } else if (!(IRPIN & (1 << IR_L))) {
    levelmeter(LINESENSOR_L);		// magnitude meter for L line sensor
  } else if (!(IRPIN & (1 << IR_R))) {
    levelmeter(LINESENSOR_R);		// magnitude meter for R line sensor
  } else {
    lineResponseMeter();		// 8-bit meter for all line sensors
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



// sets up ADC for use:
// - setup ADC Multiplexor Select register (ADMUX):
//   - select Vref (with REFSx). For the Sumovore using AVCC
//     with external capacitor at AREF pin, select this with REFS0
//   - (optional) select left adjust (ADLAR) if only want to use
//     the most significant 8 bits of ADC conversions. This is
//     suitable for the edge detectors, as only testing for the
//     magnitude of their signal, higher precision won't help.
// - setup ADC Ctrl & Status register (ADCSRA):
//   - the clk prescaler needs to be set (with ADPSx), to divide the
//     CPU clk frequency to give the ADC clk a frequency of 50-200 kHz
//     for 10-bit precision (a higher freq can be used for lower
//     precision and higher sample rates). The Sumovore uses a CPU clk
//     of 1 MHz, so a prescaler of 16 (ADPS2) is sufficient, giving
//     an ADC clk frequency of 62.5 kHz
//   - enable ADC with ADEN bit
// - first ADC conversion (after ADEN set) takes 25 ADC clk
//   cycles, after which ADC conversions take 13 clk cycles.
//   so do an initial (dummy) conversion before actual use.
void setupADC()
{
  // REFS0 (Vref) is AVCC
  // ADLAR=1 as only want magnitude of reflectivity
  ADMUX = (1 << REFS0) | (1 << ADLAR);

  // clk prescaler of 16 (ADPS2) for ADC clk (of 62.5 kHz)
  // and enable ADC with ADEN flag
  ADCSRA = (1 << ADPS2) | (1 << ADEN);

  // do a dummy conversion to complete initialization
  readADC(0);
}



// countdown from 1-5 seconds
void countdown()
{
  // ensure all red leds are off
  redsoff();

  for (int i=0; i<5; i++) {
    REDPORT |= (1 << (PD2+i));	// light red LED (PD2-6 defined as 2-6)
    waitsecs(1);
  }

  // and now turn them all off again
  redsoff();
}



// test IR detectors - indicate if both are detecting something, with the
// center red LED, or if left or right are detecting with left & right LEDs
// note that IRs return 0 when detect an obstacle, 1 otherwise
void testIRdetectors()
{
  while(1) {
    if (!(IRPIN & (1 << IR_L)) && !(IRPIN & (1 << IR_R))) { // both detect
      REDPORT |= (1 << REDLED_C);
      REDPORT &= ~((1 << REDLED_L) | (1 << REDLED_R));
    } else if (!(IRPIN & (1 << IR_L))) {		    // left detects
      REDPORT |= (1 << REDLED_L);
      REDPORT &= ~((1 << REDLED_C) | (1 << REDLED_R));
    } else if (!(IRPIN & (1 << IR_R))) {		    // right detects
      REDPORT |= (1 << REDLED_R);
      REDPORT &= ~((1 << REDLED_C) | (1 << REDLED_L));
    } else {
      REDPORT &= ~(1 << REDLED_L) | ( (1 << REDLED_C) | (1 << REDLED_R));
    }
  }
}



// use Sumovore as a reflectivity level meter for measuring the response
// of the line sensors to different backgrounds, black lines, etc
//
// flash red led indicating which sensor will be read
// continuously loop:
//   read sensor and display magnitude of level
void levelmeter(uint8_t channel)
{
  // using 8 bits as ADC is only returning high 8-bits of conversion
  uint8_t    linesensor;

  flashred(REDLED_L+channel,5);	// channel 0 -> left, .. 4 -> right

  for (;;) {
    linesensor=readADC(channel);	// read sensor
    outlevel(getlevel(linesensor));	// display magnitude of reading
  }
}



// convert a (sensor reading) value to a (signal strength) level
uint8_t getlevel(uint8_t value)
{
  uint8_t level=0;
  while (value) {
    level++;
    value >>= 1;
  }

  return level;
}



// measure the response of the line sensors, displaying the full 8-bit
// values for each (0-225), with lower values corresponding to lighter
// (reflective) surfaces, while higher values correspond to darker
// (absorbing) surfaces
//
// continuously loop:
//   for each line sensor (L to R)
//     flash red LED to indicate which sensor is being read
//     read line sensor
//     display 8-bit signal value with red LEDs (unused red LED flashes):
//   		hi 4 bits (green LED on), pause,
//   		lo 4 bits. 
void lineResponseMeter()
{
  // using 8 bits as ADC is only returning high 8-bits of conversion
  uint8_t linesensor;
  uint8_t i;

  for (;;) {
    for (i=0; i<5; ++i) {			// for each sensor
      flashred(REDLED_L+i,2);			// indicate sensor with red LED
      linesensor = readADC(LINESENSOR_L+i);	// read sensor
      displayvalue(linesensor,5);		// display reading
    }
  }
}



// uses the 5 red LEDs to output a sensor's level from 0-8
// off=0, left -> right = level 1-5, right -> left = level 6-8
// utilises PDx = x (defined in avr/iom8.h), with L-R LEDS being x=2 .. 6 ) 
void outlevel(uint8_t level)
{
  redsoff();			// start with all off = 0

  if (level<6) {		// light from left to right for levels 1-5
    while (level--) {
      PORTD |= (1 << (level+PD2));
    }
  } else {			// >5 light from right to left for 6-8
    level-=5;
    while (level--) {
      PORTD |= (1 << (PD6-level));
    }
  }
}



// display value's bit pattern 4 bits at a time with the red LEDs
// hi 4, pause, lo 4 (with green LED lit to indicate hi 4)
void displayvalue(uint8_t val, uint8_t pause)
{
  uint8_t i;

  // display high 4 bits
  redsoff();
  GREENPORT |= (1 << GREENLED);	// indicate displaying high 4 bits
  for (i=0; i<4; i++) {
    if (val & (1 << 7)) {	// check if highest bit (bit 7) set
      // noting that PD2 is leftmost and PDx is defined as x in avr/iom8.h
      PORTD |= (1 << (PD2+i));
    }
    val <<= 1;			// shift out highest bit
  }
  flashred(REDLED_R,pause);	// indicates pause, and not to read this led
  GREENPORT &= ~(1 << GREENLED);	// turn off hi 4-bit indicator LED

  // display low 4 bits
  redsoff();
  for (i=0; i<4; i++) {
    if (val & (1 << 7)) {	// check if highest bit (bit 7) set
      // noting that PD2 is leftmost and PDx is defined as x in avr/iom8.h
      PORTD |= (1 << (PD2+i));
    }
    val <<= 1;			// shift out highest bit
  }
  flashred(REDLED_R,pause);	// indicates pause, and not to read this led
  redsoff();			// clear LEDs before leaving function
}



// flash green LED for so many seconds
void flashgreen(uint8_t secs) 
{
  // convert secs to 250MS, as flash 4 times a second
  secs <<= 2;
  while(secs--) {
    GREENPORT ^= _BV(GREENLED);
    waitMS(250);
  }
}



// flash red LED for so many seconds, ledno is from 2-6 (left to right)
void flashred(uint8_t ledno, uint8_t secs) 
{
  // convert secs to 250MS, as flash 4 times a second
  secs <<= 2;

  while(secs--) {
    REDPORT ^= _BV(ledno);	// cheat! as avr/iom8.h defines PDx as x
    waitMS(250);
  }
}



// turn off all the red leds
void redsoff()
{
  REDPORT &= ~( _BV(REDLED_L) | _BV(REDLED_ML)
               | _BV(REDLED_C) | _BV(REDLED_MR)
               | _BV(REDLED_R));
}



// read from ADC channel, and return most significant 8 bits of conversion:
//  - uses same ADMUX settings as in setupADC() (see above), but also
//    select ADC channel ADC0-ADC5 (which use pins PC0-PC5)
//  - start ADC conversion by setting ADSC flag in ADCSRA, noting that
//	- ADC detects ADCSR settings on next ADC clk rising edge
//	- sample and hold then starts after 1.5 ADC clks
//	- complete conversion takes 13 ADC clks (including sample and hold)
//  - then wait for conversion to complete, indicated by ADSC flag being reset
//  - read most significant 8 bits of conversion result in ADCH
uint8_t readADC(uint8_t channel)
{
  // as not re-setting ADMUX settings, need to clear out the previous channel
  // setting (low 4 bits) first, and then set the new ADC channel
  ADMUX  &= ~0xF;
  ADMUX  |= channel;

  ADCSRA |= (1 << ADSC);		// start conversion
  waitUS(16);				// wait 1 ADC clk cycle, and then
  while (ADCSRA & (1 << ADSC));		// wait for conversion to complete

  // return most significant 8 bits of conversion
  return ADCH;
}



// wait in a loop for the specified number of seconds
void waitsecs(uint8_t secs)
{
  while(secs--) { waitMS(1000); }
}



// uses wait in a loop for the specified millisecs
void waitMS(uint16_t msec)
{
  while(msec--) { waitUS(1000); }
}



// Spin-wait in a loop for the specified number of microseconds
// uses inline Asm
//
// the "volatile" keyword avoids compiler optimising away the code
// asm statement is given in the general format of
// asm(code : output operand list : input operand list [: clobber list]);
//
//   code: "1: sbiw %0,1" "\n\t" "brne 1b"
//   		%0 is first operand (cnt)
//   		\n\t is linefeed tab for readability in generated code
//   output operand list: "=w" (cnt)
//   		operand: cnt
//   		modifiers:
//   			'=' write-only operand (used for all output operands)
//   			'w' use special upper register pairs (r24,r26,r28,r30)
//
//   input operand list: "0" (cnt)
//   		operand: cnt
//   		modifiers: '0' use the same input register as first operand
//
// generated asm code is:
//
// 	1: sbiw r30, 1
// 	brne 1b
//
//   "1:" is a Unix-assembler style local numeric label
//   "sbiw" reg, const: Subtract Immediate (1) from Word (cnt)
//   "brne" label: branch if not equal to label "1" back ('b')
//
void waitUS(uint16_t usec)
{
  register uint16_t cnt;

  // the ASM code takes 4 CLK cycles per loop
  // so for a 1MHz CLK divide by 4, and scale this up for higher CLK rates
  cnt = (usec * (MCU_FREQ / 1000000UL))  / 4;

  if (!cnt) { return; } // otherwise sbiw will decrement cnt to 0xFFFF

  __asm__ volatile (
      "1: sbiw %0,1" "\n\t"
      "brne 1b"
      : "=w" (cnt)
      : "0"  (cnt)
  );
}




