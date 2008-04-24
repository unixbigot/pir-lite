/*
 * Simple LED nightlight timer 
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

register int8_t  pwm_step asm("r2"); // brightening, dimming, or steady
register int8_t  butn_debounce asm("r3");
register int16_t cnt_jiffies asm("r4"); // time remaining before off (in jiffies, 250ms)

#define HZ		4

#define ON_SECS		300
#define UP_SECS_FAST	1
#define UP_SECS_SLOW	4
#define OFF_SECS_SLOW	64
#define OFF_SECS_FAST	16

#define SECSTOSTEP(secs) (256/(secs * HZ));

#ifdef __AVR_ATtiny45__
/* 
 *@@ Pin assignments for ATtiny45
 * 
 * LITE - MOSFET driving LED array via PWM on internal timer 0
 * SENS - PIR sensor input
 * BUTN - manual override button
 * TICK - timer tick from internal timer 1 (clk/16384) = 8MHz/16k + TOP=122 => 4.0023 Hz
 */

#define LED_OCR		OCR0A
#define LED_OVF_VECT 	TIM0_OVF_vect
#define LED_COM_MODE	_BV(COM0A1)
#define LED_WGM		_BV(WGM00)
#define LED_CLK_MODE	_BV(CS02)
#define LED_DDR		DDRB
#define LED_PORT	PORTB
#define LED_BIT		PB0

#define TICK_OCR	OCR1C
#define TICK_TOP	122
#define TICK_CLK_MODE   _BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10)
#define TICK_OVF_VECT	TIM1_OVF_vect

// sensor uses PB2/INT0 edge triggered interrupt
#define SENS_DDR	DDRB
#define SENS_PORT	PORTB
#define SENS_PIN	PINB
#define SENS_BV		_BV(PB2)
#define SENS_VECT	INT0_vect
#define SENS_IEF	INT0

// button uses PB1 pin-change interrupt
#define BUTN_DDR	DDRB
#define BUTN_PORT	PORTB
#define BUTN_PIN	PINB
#define BUTN_BV		_BV(PB1)
#define BUTN_VECT	PCINT0_vect
#define BUTN_IEF	PCIE

// pins PB5 and PB4 are spare.   We could use pb4 for a bit-bang diagnostic UART if requried.
#define HELLO_BV	_BV(PB5)
#define HELLO_DDR	DDRB
#define HELLO_PORT	PORTB
#define HELLO_PIN	PINB

#else
#error unsupported chip
#endif

/* 
 *@ Interrupt Service routine for system clock
 */
ISR(TICK_OVF_VECT)	
{
	/* 
	 * If the LED is on, reduce the on-time by one jiffy
	 */
	if (cnt_jiffies) {
		--cnt_jiffies;
		if (cnt_jiffies == 0) {
			// switch to dimming mode (slow one minute dim)
			pwm_step = 0-SECSTOSTEP(OFF_SECS_SLOW);
			if (pwm_step == 0)
				pwm_step = -1;
		}
	}

	if (butn_debounce) {
		--butn_debounce;
		if (butn_debounce == 0) {
			// re-enable the button interrupt
			GIMSK &= ~_BV(BUTN_IEF);
		}
	}
	
	/* 
	 * If we are brightening, bump the brightness by the step factor
	 */
	if (pwm_step > 0) {
		int16_t pwm = LED_OCR;
		pwm += pwm_step;
		if (pwm >= 255)
		{
			// we hit max brightness, stablize and start the off-timer
			pwm = 255;
			pwm_step = 0;
			// set a timer until turn-off time
			cnt_jiffies = ON_SECS*HZ;
		}
		LED_OCR=pwm;
	}
	else if (pwm_step < 0) {
		int16_t pwm = LED_OCR;
		pwm -= pwm_step;
		if (pwm <= 0)
		{
			// we hit OFF state, go idle
			pwm = 0;
			pwm_step = 0;
		}
		LED_OCR = pwm;
	}

	// flip the hello world LED
	HELLO_PIN |= HELLO_BV;
}

/* 
 *@ ISR for sensor input (INT0)
 * 
 */
ISR(SENS_VECT)
{
	if (!LED_OCR) {
		pwm_step = SECSTOSTEP(UP_SECS_SLOW);
		if (pwm_step < 1)
			pwm_step = 1;
	}
}

/* 
 *@ ISR for button input (PCINT0)
 * 
 */
ISR(BUTN_VECT)
{
	/* 
	 * Ignore spurious interrupts during debounce period
	 */
	if (butn_debounce)
		return;

	/* 
	 * Disable interrupt, set timer to re-enable in 1 second
	 */
	GIMSK &= _BV(BUTN_IEF);
	butn_debounce = 4; 
	
	/* 
	 * If LED is off, fade it up fast
	 * If LED is on, fade it out relatively quickly
	 */
	if (!LED_OCR) {
		pwm_step = SECSTOSTEP(UP_SECS_FAST); // 1s fade up
		if (pwm_step == 0) 
			pwm_step = 1;
	}
	else {
		pwm_step = 0-SECSTOSTEP(OFF_SECS_FAST); // 16s fade down
		if (pwm_step == 0)
			pwm_step = -1;
	}
}

/* 
 *@ IO init 
 */
void
ioinit (void)			/* Note [6] */
{
	char i;
	
#ifdef HELLO_BV
	/* 
	 * Turn on a LED used for diagnostics
	 */
	HELLO_DDR |= HELLO_BV;
	HELLO_PORT |= HELLO_BV; 
	for (i=0;i<100;i++) delay_ms(10);
#endif

	/* 
	 * Set up the timer 0 OC pin as as PWM output
	 *
	 * Configure timer 0 to Intialize to mode 1 (phase correct PWM), non-inverted, timer stopped
	 */
	TCCR0A = LED_COM_MODE|LED_WGM;
	TCCR0B = 0;
	
	/* Set initial PWM value to 0. */
	LED_OCR = 0;

	/* set the PWM output-compare pin as output */
	LED_DDR |= _BV(LED_BIT);

	/* 
	 * Start timer 0.
	 *
	 * Set clock source to IOclk/64
	 */
	TCCR0B |= LED_CLK_MODE;

	/* 
	 * Set up timer 1 as clock tick source
	 */
	TCCR1 |= _BV(PWM1A); // clear timer on OCR1C match, generate overflow interrupt
	TCCR1 |= TICK_CLK_MODE; // prescaler divider 16384
	TICK_OCR = TICK_TOP;
	TIMSK |= _BV(TOIE1); // overflow interrupt enable 
 
	/* 
	 * Set up sensor input
	 * 
	 * Sensor is active logic, HIGH level when triggered
	 */
	MCUCR |= _BV(ISC01)|_BV(ISC00); // interrupt on rising edge
	GIMSK |= _BV(SENS_IEF);		// INT0 enable
	
	/* 
	 * Set up button input
	 *
	 * Simple pushbutton with internal pullup and some software debouncing
	 */
	BUTN_PORT |= BUTN_BV; // enable pullup
	PCMSK |= BUTN_BV;
	GIMSK |= _BV(BUTN_IEF);

	sei ();
}

int
main (void)
{
	ioinit ();
	// led off initially
	pwm_step = 0;
	cnt_jiffies = 0;
	butn_debounce = 0;
	
	/* loop forever, the interrupts are doing the rest */
	for (;;)
		sleep_mode();

	return (0);
}
