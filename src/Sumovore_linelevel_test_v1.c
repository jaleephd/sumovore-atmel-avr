
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>


uint16_t ADCIN( uint8_t channel );
void     DELAY_US( uint16_t microseconds );
void     DELAY_MS( uint16_t milliseconds );

void levelmeter(uint8_t channel);
void levelstest(void);
void countdown(void);
void redsoff(void);
void outlevel(uint8_t level);
void displayvalue(uint8_t value, uint8_t pause);
void flashgreen(uint8_t secs);
void flashred(uint8_t ledno, uint8_t secs);
void waitsecs(uint8_t secs);
uint8_t getlevel(uint8_t value);


#define MCU_FREQ 1000000UL

#define GREENLED	PB3

// Left to Right red LEDs (looking from the back of the sumovore)
#define REDLED_L	PD2
#define REDLED_ML	PD3
#define REDLED_C	PD4
#define REDLED_MR	PD5
#define REDLED_R	PD6

#define LMOTOR_EN	PB1
#define RMOTOR_EN	PB2
#define LMOTOR_DIR	PB4
#define RMOTOR_DIR	PB5


int main( void )
{
    /* turn off all interrupts */

    cli();
    TIMSK = 0;

    /* the sumovore atmel brainboard has five red leds and one green led
       that we can use to display things in our program; we need to set
       the portb and portd data direction registers to enable output for 
       those pins */

    DDRB = (1 << GREENLED);

    DDRD = (1 << REDLED_L) | (1 << REDLED_ML)
         | (1 << REDLED_C) | (1 << REDLED_MR)
         | (1 << REDLED_R);

    /* the sumovore has two motors that also require output pins */

    DDRB |= ( 1 << LMOTOR_EN ) | ( 1 << RMOTOR_EN )
         | ( 1 << LMOTOR_DIR ) | ( 1 << RMOTOR_DIR );

    // not using the motors, so turn them off via left and right motor enable
    PORTB &= ~((1 << LMOTOR_EN) | (1 << RMOTOR_EN));

    /* enable the ADC and start the first (dummy) conversion to complete 
       the initialization */

    ADCSRA = ( 1 << ADEN  ) | ( 1 << ADSC  ) | ( 1 << ADPS2 ); // 62.5 kHz

    while ( ADCSRA & ( 1 << ADSC ) ); /* wait for complete conversion */

    // use the bot as a edge sensor level meter.
    // argument specifies ADC channel (0-4 for left - right)
    levelmeter(0);

    // uncomment the following to run levels test
    //levelstest();

    /* this point of the program is never reached, but we satisfy the
       compiler's warnings by saying we'd return zero */

    return 0;
}


// flash red led indicating which sensor will be read
// continuously loop:
//   read sensor and display magnitude of level
void levelmeter(uint8_t channel)
{
  // using 8 bits as ADC is only returning high 8-bits of conversion
  uint8_t    linesensor;

  flashred(REDLED_L+channel,5);	// channel 0 -> left, .. 4 -> right

  for (;;) {
    linesensor=(uint8_t)ADCIN(channel);	// read sensor
    outlevel(getlevel(linesensor));	// display magnitude of reading
  }
}


// continuously loop:
//   5 second countdown to place robot;
//   read signals for left, midleft, center, midright, right line sensors
//   display with red LEDs
//   	level of signal magnitude (0-8)
//   		off=0, left -> right = level 1-5, right -> left = level 6-8
//   	actual signal values (8-bit)
//   		hi 4 bits (green LED on), pause, lo 4 bits. unused LED flashes
void levelstest()
{
  // using 8 bits as ADC is only returning high 8-bits of conversion
  uint8_t    linesensor_left;
//  uint8_t    linesensor_midleft;
//  uint8_t    linesensor_center;
//  uint8_t    linesensor_midright;
//  uint8_t    linesensor_right;
  uint8_t    lvl_left;
//  uint8_t    lvl_midleft;
//  uint8_t    lvl_center;
//  uint8_t    lvl_midright;
//  uint8_t    lvl_right;

  for (;;) {
    countdown();

    // note: sensors return a value from 0 - 255
    // corresponding to lighter (reflective) to darker (absorbing) surface

    // get outer left edge
    flashred(REDLED_L,2);			// indicate reading left sensor
    linesensor_left = (uint8_t)ADCIN(0);
    lvl_left=getlevel(linesensor_left);		// get magnitude of reading
    outlevel(lvl_left);				// and display it
    waitsecs(5);
    //outlevel(0);
    displayvalue(linesensor_left,5);		// display actual reading

    // uncomment sensors that want to test
/*
    // get midleft
    flashred(REDLED_ML,2);			// indicate midleft sensor
    linesensor_midleft = (uint8_t)ADCIN(1);
    lvl_midleft=getlevel(linesensor_midleft);	// get magnitude of reading
    outlevel(lvl_midleft);			// and display it
    waitsecs(5);
    displayvalue(linesensor_midleft,5);		// display actual reading

    // get center
    flashred(REDLED_C,2);			// indicate center sensor
    linesensor_center = (uint8_t)ADCIN(2);
    lvl_center=getlevel(linesensor_center);	// get magnitude of reading
    outlevel(lvl_center);			// and display it
    waitsecs(5);
    displayvalue(linesensor_center,5);		// display actual reading

    // get midright
    flashred(REDLED_MR,2);			// indicate midright sensor
    linesensor_midright = (uint8_t)ADCIN(3);
    lvl_midright=getlevel(linesensor_midright);	// get magnitude of reading
    outlevel(lvl_midright);			// and display it
    waitsecs(5);
    displayvalue(linesensor_midright,5);	// display actual reading

    // get right
    flashred(REDLED_R,2);			// indicate right sensor
    linesensor_left = (uint8_t)ADCIN(4);
    lvl_right=getlevel(linesensor_right);	// get magnitude of reading
    outlevel(lvl_right);			// and display it
    waitsecs(5);
    displayvalue(linesensor_right,5);		// display actual reading
*/
  }
}


// countdown from 1-5
void countdown()
{
  // ensure all red leds are off
  redsoff();

  PORTD |= (1 << REDLED_L);
  waitsecs(1);
  PORTD |= (1 << REDLED_ML);
  waitsecs(1);
  PORTD |= (1 << REDLED_C);
  waitsecs(1);
  PORTD |= (1 << REDLED_MR);
  waitsecs(1);
  PORTD |= (1 << REDLED_R);
  waitsecs(1);

  // and now turn them all off again
  redsoff();
}


// display bit pattern of value 4 bits at a time
// hi 4, pause, lo 4 (with green LED lit to indicate hi 4)
void displayvalue(uint8_t val, uint8_t pause)
{
  uint8_t i;

  // display high 4 bits
  redsoff();
  PORTB |= (1 << GREENLED);	// indicate displaying high 4 bits
  for (i=0; i<4; i++) {
    if (val & 128) {
      // noting that PD2 is leftmost and PDx is defined as x in avr/iom8.h
      PORTD |= (1 << (2+i));
    }
    val <<= 1;			// shift out highest bit
  }
  flashred(REDLED_R,pause);	// indicates pause, and not to read this led
  PORTB &= ~(1 << GREENLED);	// turn off hi 4-bit indicator LED

  // display low 4 bits
  redsoff();
  for (i=0; i<4; i++) {
    if (val & 128) {
      // noting that PD2 is leftmost and PDx is defined as x in avr/iom8.h
      PORTD |= (1 << (2+i));
    }
    val <<= 1;			// shift out highest bit
  }
  flashred(REDLED_R,pause);	// indicates pause, and not to read this led
  redsoff();

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


// flash green LED for so many seconds
void flashgreen(uint8_t secs) 
{
  // convert secs to 250MS, as flash 4 times a second
  secs <<= 2;
  while(secs--) {
    PORTB ^= (1 << GREENLED);
    DELAY_MS(250);
  }
}


// flash red LED for so many seconds, ledno is from PD2-PD6 (left to right)
void flashred(uint8_t ledno, uint8_t secs) 
{
  // convert secs to 250MS, as flash 4 times a second
  secs <<= 2;

  while(secs--) {
    PORTD ^= (1 << ledno); // cheat! as avr/iom8.h defines PDx as x
    DELAY_MS(250);
  }
}


// turn off all the red leds
void redsoff()
{
  PORTD &= ~( (1 << REDLED_L) | (1 << REDLED_ML)
            | (1 << REDLED_C) | (1 << REDLED_MR)
            | (1 << REDLED_R));
}


void waitsecs(uint8_t secs)
{
  while(secs--) {
    DELAY_MS(1000);		// 1000 millisec = 1 sec
  }
}


/*
 *  ADCIN
 *
 *  Read the specified Analog to Digital Conversion channel
 *
 */

uint16_t ADCIN( uint8_t channel )
{
    /* set for 8-bit results for the desired channel number then start the
       conversion; pause for the hardware to catch up */
    
    ADMUX  = ( 1 << ADLAR ) | ( 1 << REFS0 ) | channel;
    ADCSRA |= ( 1 << ADSC  );
    DELAY_US( 16 ); // for prescaler of 64 use: DELAY_US( 64 ); 

    /* wait for complete conversion and return the result */

    while ( ADCSRA & ( 1 << ADSC ) );
    
    return ADCH;
}


/*
 *  DELAY_US
 *
 *  Spin-wait in a loop for the specified number of microseconds.
 *
 */

void DELAY_US( uint16_t microseconds )
{
    register uint16_t loop_count;

#if MCU_FREQ == 8000000UL

    /* 8mhz clock, 4 instructions per loop_count  */
    loop_count = microseconds * 2;

#elif MCU_FREQ == 1000000UL

    /* 1mhz clock, 4 instructions per loop_count */
    loop_count = microseconds / 4;

#else
#error MCU_FREQ undefined or set to an unknown value!
    loop_count = 0; /* don't really know what to do */
#endif

    __asm__ volatile (
        "1: sbiw %0,1" "\n\t"
        "brne 1b"
        : "=w" ( loop_count )
        : "0"  ( loop_count )
    );
}


/*
 *  DELAY_MS
 *
 *  Spin-wait in a loop for the specified number of milliseconds.
 *
 */

void DELAY_MS( uint16_t milliseconds )
{
    uint16_t i;

    for ( i = 0; i < milliseconds; ++i )
    {
        DELAY_US( 1000 );
    }
}


