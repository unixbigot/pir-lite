/*
 * Simple LED nightlight timer 
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

register int8_t  pwm_step asm("r2"); // brightening, dimming, or steady
register uint8_t  butn_debounce asm("r3");
register uint16_t cnt_jiffies asm("r4"); // time remaining before off (in jiffies, 250ms)

#define HZ		16

#define ON_SECS		300
#define UP_SECS_FAST	1
#define UP_SECS_SLOW	4
#define OFF_SECS_SLOW	16
#define OFF_SECS_FAST	4

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

#define LED_OCR			OCR0A
#define LED_OVF_VECT		TIM0_OVF_vect
#define LED_COM_MODE		_BV(COM0A1)
#define LED_WGM			_BV(WGM00)
#define LED_CLK_MODE		_BV(CS01)
#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_BIT			PB0
#define LED_TOP			255
#define LED_BOT			1

#define TICK_OCR		OCR1C
#define TICK_TOP		122
#define TICK_CLK_MODE		_BV(PWM1A)
#define TICK_CLK_DIV_1024	(_BV(CS13)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV_4096	(_BV(CS13)|_BV(CS12)|_BV(CS10))
#define TICK_CLK_DIV_16384	(_BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV		TICK_CLK_DIV_4096
#define TICK_OVF_VECT		TIM1_OVF_vect

// sensor uses PB2/INT0 edge triggered interrupt
#define SENS_DDR		DDRB
#define SENS_PORT		PORTB
#define SENS_PIN		PINB
#define SENS_BV			_BV(PB2)
#define SENS_VECT		INT0_vect
#define SENS_IEF		INT0
#define SENS_MSK		GIMSK

// button uses PB1 pin-change interrupt
#define BUTN_DDR		DDRB
#define BUTN_PORT		PORTB
#define BUTN_PIN		PINB
#define BUTN_BV			_BV(PB1)
#define BUTN_VECT		PCINT0_vect
#define BUTN_IEF		PCIE
#define BUTN_MSK		PCMSK

// pins PB5 and PB4 are spare.   We could use pb4 for a bit-bang diagnostic UART if requried.
#define HELLO_BV		_BV(PB3)
#define HELLO_DDR		DDRB
#define HELLO_PORT		PORTB
#define HELLO_PIN		PINB


#elif defined(__AVR_ATmega8515__) 
/* 
 * Pin assignments for ATmega8515 (STK 200)
 *
 * Note: this is NOT TESTED as I no longer have access to an STK200
 */

// LED uses PB0/OC0 (pin 1)
#define LED_OCR			OCR0
#define LED_OVF_VECT		TIM0_OVF_vect
#define LED_COM_MODE		_BV(COM0A1)
#define LED_WGM			_BV(WGM00)
#define LED_CLK_MODE		_BV(CS01)
#define LED_DDR			DDRB
#define LED_PORT		PORTB
#define LED_BIT			PB0
#define LED_TOP			255
#define LED_BOT			1

// Clock timer uses (16 bit) timer 1 
#define TICK_OCR		OCR1AL
#define TICK_TOP		122
#define TICK_OCRH		OCR1AH
#define TICK_TOPH		0
#define TICK_CLK_MODE		_BV(PWM1A)
#define TICK_CLK_DIV_1024	(_BV(CS13)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV_4096	(_BV(CS13)|_BV(CS12)|_BV(CS10))
#define TICK_CLK_DIV_16384	(_BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10))
#define TICK_CLK_DIV		TICK_CLK_DIV_4096
#define TICK_OVF_VECT		TIM1_OVF_vect

// sensor uses PD2/INT0 edge triggered interrupt (pin 12)
#define SENS_DDR		DDRD
#define SENS_PORT		PORTD
#define SENS_PIN		PIND
#define SENS_BV			_BV(PD2)
#define SENS_VECT		INT0_vect
#define SENS_IEF		INT0
#define SENS_MASK		GICR

// button uses PD3/INT1 pin-change interrupt (pin 13)
#define BUTN_DDR		DDRD
#define BUTN_PORT		PORTD
#define BUTN_PIN		PIND
#define BUTN_VECT		INT1_vect
#define BUTN_IEF		INT1
#define BUTN_MASK		GICR

// test LED on Pin 2 (PB1)
#define HELLO_BV		_BV(PB1)
#define HELLO_DDR		DDRB
#define HELLO_PORT		PORTB
#define HELLO_PIN		PINB

#else
#error unsupported chip
#endif

/* 
 *@ Interrupt Service routine for system clock
 */
ISR(TICK_OVF_VECT)	
{
	int16_t pwm;
	/* 
	 * If the LED is on, reduce the on-time by one jiffy
	 */
	if (cnt_jiffies != 0) {
		--cnt_jiffies;
		if (cnt_jiffies == 0) {
			// switch to dimming mode (slow one minute dim)
			pwm_step = 0-SECSTOSTEP(OFF_SECS_SLOW);
			if (pwm_step == 0)
				pwm_step = -1;
		}
	}

	if (butn_debounce > 0) {
		--butn_debounce;
		if (butn_debounce == 0) {
			// re-enable the button interrupt
			BUTN_MSK |= _BV(BUTN_IEF);
			SENS_MSK |= _BV(SENS_IEF);
		}
	}

	/* 
	 * If led is stable, bail
	 */
	if (pwm_step == 0)
		return;
	
	/* 
	 * Alter the brightness brightness by the step factor
	 *
	 * If we are brightening, check for LED_TOP
	 * If we are dimming, check for LED_BOT
	 */
	pwm = LED_OCR + pwm_step;
	if (pwm_step > 0) {
		if (pwm >= LED_TOP)
		{
			// we hit max brightness, stablize and start the off-timer
			pwm = LED_TOP;
			pwm_step = 0;
			// set a timer until turn-off time
			cnt_jiffies = ON_SECS*HZ;
		}
	}
	else /* if (pwm_step < 0)*/ {
		/* 
		 * We are dimming, reduce the brightness
		 */
		if (pwm <= LED_BOT)
		{
			// we hit min brightness, go idle
			pwm = LED_BOT;
			pwm_step = 0;
			// reenable sensor interrupt 
			SENS_MSK |= _BV(SENS_IEF);
		}
	}
	LED_OCR = pwm;
}

/* 
 *@ ISR for sensor input (INT0)
 * 
 */
ISR(SENS_VECT)
{
	if (LED_OCR <= LED_BOT) {
		pwm_step = SECSTOSTEP(UP_SECS_SLOW);
		if (pwm_step < 1)
			pwm_step = 1;
	}
	SENS_MSK |= _BV(SENS_IEF);
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
	if (butn_debounce != 0)
		return;

	/* 
	 * Disable interrupt, set timer to re-enable in ~250ms
	 */
	BUTN_MSK &= _BV(BUTN_IEF);
	butn_debounce = 1+HZ/4;  // at least a quarter second debounce
	
	/* 
	 * If LED is fading on/off stop at current brightness
	 * If LED is full off, fade it up fast
	 * If LED is full on, fade it out relatively slowly
	 * If LED is half on, fade it out relatively quickly
	 */

	if (pwm_step != 0) {
		/* 
		 * Pause fade at current brightness
		 */
		pwm_step = 0;
		return;
	}
	
	if (LED_OCR <= LED_BOT) {
		/* 
		 * Turn LED on
		 */
		pwm_step = SECSTOSTEP(UP_SECS_FAST); // 1s fade up
		if (pwm_step == 0) 
			pwm_step = 1;
	}
	else {
		/* 
		 * Turn led off quick (manual off, or resume a paused fade-out)
		 */
		pwm_step = 0-SECSTOSTEP(OFF_SECS_FAST); // slow fade (about 4s from full to off)
		if (pwm_step == 0)
			pwm_step = -1;
		cnt_jiffies = 0; // disable off-timer
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
	for (i=0;i<100;i++) _delay_ms(10);
	HELLO_PORT &= ~(HELLO_BV); 

#endif
	LED_OCR=0x80;

	/* 
	 * Set up the timer 0 OC pin as as PWM output
	 *
	 * Configure timer 0 to Intialize to mode 1 (phase correct PWM), non-inverted, timer stopped
	 */
#ifdef TCCR0
	TCCR0A = LED_COM_MODE|LED_WGM;
#else
	TCCR0A = LED_COM_MODE|LED_WGM;
	TCCR0B = 0;
#endif
	
	/* Set initial PWM value to 0. */
	LED_OCR = 0;

	/* set the PWM output-compare pin as output */
	LED_DDR |= _BV(LED_BIT);

	/* 
	 * Start timer 0.
	 *
	 * Set clock source to IOclk/64
	 */
#ifdef TCCR0
	TCCR0 |= LED_CLK_MODE;
#else
	TCCR0B |= LED_CLK_MODE;
#endif

	/* 
	 * Set up timer 1 as clock tick source
	 */
#ifdef TCCR1
	TCCR1 |= TICK_CLK_MODE ; // clear timer on OCR1C match, generate overflow interrupt
	TCCR1 |= TICK_CLK_DIV; // prescaler divider 16384
#else
	TCCR1A |= TICK_CLK_MODE); // clear timer on OCR1C match, generate overflow interrupt
	TCCR1B |= TICK_CLK_DIV; // prescaler divider 16384
#endif
	TICK_OCR = TICK_TOP;
#ifdef TICK_OCRH
	TICK_OCRH = TICK_TOPH;
#endif
	TIMSK |= _BV(TOIE1); // overflow interrupt enable 
 
	/* 
	 * Set up sensor input
	 * 
	 * Sensor is active logic, HIGH level when triggered
	 */
	MCUCR |= _BV(ISC01)|_BV(ISC00); // interrupt on rising edge
	SENS_MSK |= _BV(SENS_IEF);		// INT0 enable
	
	/* 
	 * Set up button input
	 *
	 * Simple pushbutton with internal pullup and some software debouncing
	 */
	BUTN_DDR &= ~BUTN_BV;   // input
	BUTN_PORT |= BUTN_BV;   // enable pullup
#if defined(PCMSK) &&  (BUTN_MSK == PCMSK)
	PCMSK |= BUTN_BV;	// pin change interrupt on PB1
#endif
	BUTN_MSK |= _BV(BUTN_IEF);

	sei ();
}

int
main (void)
{
	// led off initially
	pwm_step = 0;
	cnt_jiffies = 0;
	butn_debounce = HZ; // ignore button interrupt during power on
	

	ioinit ();
	butn_debounce = 0; // allow button input
	
	/* loop forever, the interrupts are doing the rest */
	for (;;)
		sleep_mode();

	return (0);
}
