#define F_CPU 16000000UL

#include <stdio.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*** Local includes ***/
#include "LED.c"
#include "Serial.c"

#if 0
#include "Light.c"

/*** Current time ***/
struct Clock {
	unsigned char Day;
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
	int16_t Part;
} Clock = {0, 0, 0, 0, 0};

/** Schedule slicing interrupt in 1 / (16*10^6/64/OCR0) s */
static inline void TimerSchedule()
{
	TCNT0 = 0;
	TIFR |= (1<<OCF0);	/* Clear interrupt */
	TIMSK |= (1<<OCIE0);	/* Enable interrupt reception */
}

/** Turn off slicing-timer interrupt */
static inline void TimerOff()
{
	TIMSK &= ~(1<<OCIE0);
}

/* Power-line synchronization.
 * Called with f=4*50Hz, w=0.005s */
volatile uint8_t CycleDone = 1;
ISR(INT1_vect) 
{
	/* Update clock */
	Clock.Part++;
//	if (Clock.Part == (50*4)) { /* This won't work - who knows why? */
	if (Clock.Part % (4*50) ==0) {
		Clock.Part = 0;
		LEDSwitch(LED_G);
		if (++Clock.Second == 60) {
			Clock.Second = 0;
			LEDSwitch(LED_R);
			if (++Clock.Minute == 60) {
				Clock.Minute = 0;
				if (++Clock.Hour == 24) {
					Clock.Hour = 0;
					++Clock.Day;
				}
			}

		}
	}

	if (CycleDone) {
		CycleDone = 0;
		TimerSchedule();
		/* Light lamps */
		LightOn(0);
	}

	/* Turn on everything which needs to be turned on */
	
	/* For now, turn it */
//	LightDimmed(0, (uint8_t)((uint16_t)Clock.Second * 170 / 60));

}

// 99 interrupts should come
volatile uint8_t SliceCount;
ISR(TIMER0_COMP_vect)
{
	static unsigned char i;

	SliceCount++;

	if (SliceCount == 95) { /* 97 seems fine (we don't miss the next 0; but make sure) */
		SliceCount = 0;
		CycleDone = 1;
		TimerOff(); /* Synchronize with power line */
		return;
	}

	for (i = 0; i < LightCount; i++) {
		if (Lights[i].State == L_DIMMED_ON) {
			if (--Lights[i].TimeToOff == 0) {
				/* Turn off */
				LightOff(i);
				Lights[i].State = L_DIMMED_OFF;
			}
		}
	}
}

static inline void Initialize(void)
{
	/* Initialize zero detector interrupt */
//  	MCUCR = (1<<ISC11)|(1<<ISC10);	/* The rising edge of INT0 generates an interrupt request */
  	MCUCR = (1<<ISC11);		/* The falling edge of INT0 generates an interrupt request */
//	MCUCR = (1<<ISC10);		/* Any edge of INT0 generates an interrupt request */

	GICR |= (1<<INT1);
//	PORTD |= (1<<PD2);	/* Internal pull-up */

	/* Initialize time-slicing interrupt */
	// solve((1/50 / 100) = 1/(16*10^6 / 64 / x), x), numer;
	// | x   | slicing |
	// | 50  | 100     |
	// | 10  | 500     |
	// | 100 | 10      |


	TimerOff();
	OCR0 = 50;
	TCCR0 = (1<<CS01) | (1<<CS00) | (1<<WGM01); /* /64, CTC - clear timer on match */
}

void IRLedTest()
{
	/* f = 36kHz */
	/* T = 0.027ms = 27us */

	DDRA=(1<<PA0);
	
	for (;;) {
		PORTA ^= (1<<PA0);
//		_delay_us(27.0);
		_delay_us(27.0/2.0);
	}
}

#endif 


/************************************************************************************************************
 * Guitar tunner - NOTES
 *
 * FFT_N = 256
 * FFT_N / 2 = max Hz
 * Musi wej�� FFT_N/4 pe�nych okres�w danej cz�stotliwo�ci
 * f = 10^6 / prescaler / 13 / divider
 *
 *      f [Hz]   T [s]   f * (FFT_N/4)   prescaler
 * E   329.628 .0030337  21096.192       64  -> 19230 Hz
 * H   246.942 .0040495  15804.288
 * G   195.998 .0051021  
 * D   146.832 .0068105
 * A   110.000 .0090909
 * E2   82.407 .0121349   5274.048
 *
 * Formula:
 * Clock / ADCPrescaler / 13 / Divider / 2 * (BAR / 128) = Hz
 * We are trying to hit 32 BAR
 * 16*10^6 / 128 / 13 / D / 2  *  (32/128) = f 
 * D = 2403.846153 / f
 * B = 0.026624 f D
 *
 * 2403.846153 / 329.628 = 7.292603034329608 = 7
 * 2403.846153 / 246.942 = 9.734456483708724 = 10
 * 2403.846153 / 195.998 = 12.26464633822794 = 12
 * 2403.846153 / 146.832 = 16.37140509561948 = 16
 * 2403.846153 / 110.000 = 21.85314684545455 = 22
 * 2403.846153 /  82.407 = 29.17041213731843 = 29
 * Bar:
 * 0.026624 * 329.628 * 7 =  61.43211110399999
 * 0.026624 *246.942 * 10 =  65.74583808	  
 * 0.026624 *195.998 * 12 =  62.61900902399999
 * 0.026624 *146.832 * 16 =  62.54808268799999
 * 0.026624 *110.000 * 22 =  64.43007999999999
 * 0.026624 * 82.407 * 29 =  63.62611507199999
 *
 */
 /*
 *
 * E (FFT bar 65):
 * FFT_N = 256 Prescaler = 128 Divider = 5
 * Sampling freq = 1923.07692307692307692307
 * Max freq = 961.5 Hz
 * Center freq = 480.7692 Hz  --- 1.4585 * 329.628
 * 
 * H (FFT bar 66):
 * FFT_N = 256 Prescaler = 128 Divider = 7
 * Sampling freq = 1373.62637362637362637362
 * Center freq = 343.40 Hz ---  1.39 * 246.942
 *
 * G (FFT bar 68):
 * FFT_N = 256 Prescaler = 128 Divider = 9
 * Sampling freq = 1068.376068376
 * Center freq = 267.09401 Hz ---  1.36 * 246.942
 *
 * D (FFT bar 62):
 * FFT_N = 256 Prescaler = 128 Divider = 11
 * Sampling freq = 874.1258
 * Center freq = 218.5314 Hz ---  1.48830955 * 246.942
 *
 * A (FFT bar 62):
 * FFT_N = 256 Prescaler = 128 Divider = 15
 * Sampling freq = 641.02564
 * Center freq = 160.25641025 Hz ---  1.456876 * 246.942
 *
 * E2 (FFT bar 65):
 * FFT_N = 256 Prescaler = 128 Divider = 21
 * Sampling freq = 457.8754578
 * Center freq = 114.468864468 Hz ---  1.38906724 * 246.942





 *
 * Max frequency to sample = 412
 * Sampling rate >= 824 Hz
 */

#include "FFT/ffft.h"

/* FFT Buffers */
int16_t capture[FFT_N];
complex_t bfly_buff[FFT_N];
uint16_t spektrum[FFT_N/2];


volatile int capture_pos;               /* Position in buffer            */
volatile int16_t adc_sum;               /* Sum of measurements           */
volatile int16_t adc_cur;               /* Current measurement           */
volatile int16_t background;            /* Estimated Light background    */

/* Note to chase */
volatile char note_current = 0;
volatile char note_divisor = 30;
volatile char note_bar = 30;

ISR(ADC_vect) 
{
	static char i;
	static int16_t tmp;

	/* Read measurement. It will get averaged */
	tmp = ADC - 512;
	adc_sum += tmp - background;

	/* Calculate background all the time */
	background *= 7;
	background += tmp;
	background /= 8;

	/* Increment divisor, return if too small */
	if (++i % note_divisor  != 0) {
		return;
	}

  	/* Average all measurements into one */
//	adc_cur = adc_sum / note_divisor;
	adc_cur = tmp - background;
	adc_sum = 0;

	/* Ignore saving if buffer is full */
	if (capture_pos >= FFT_N)
		return;

	/* Multiply to better fit FFT */
	adc_cur *= 200;

	/* Store */
	capture[capture_pos] = adc_cur;
	++capture_pos;
}

enum { NOTE_E2 = 0, NOTE_A, NOTE_D, NOTE_G, NOTE_H, NOTE_E }; 
struct {
	int divisor;
	int bar;
} notes[] = {

	/* f=82.407 presc=64 div=31 bar=34 err=0.01711  */
	{31, 31},
	/* f=110.000 presc=64 div=28 bar=41 err=0.00258  */
	{28, 42},
	/* f=146.832 presc=64 div=22 bar=43 err=0.00617  */
	{22, 46},
	/* f=195.998 presc=64 div=23 bar=60 err=0.03228  */
	{23, 61},
	/* f=246.942 presc=64 div=14 bar=46 err=0.11851  */
	{14, 45},
	/* f=329.628 presc=64 div=13 bar=57 err=0.25485  */
	{13, 58},
	
	/* For prescaler / 128 */
/*
	{27, 56}, // Error: 0.0637 //
	{20, 40}, // Error: 0.0026 //
	{15, 42}, // Error: 0.0062 //
	{11, 45}, // Error: 0.1492 //
	{9, 43},  // Error: 0.1185 //
	{7, 40},  // Error: 0.9008 //
*/
};

void do_capture(int note)
{
	/* TESTS */
/*
	static int done = 0;
	int i;
	if (done) return;

	for (i=0; i<FFT_N; i++) { 
		capture[i] = 3000.0*sin(((float)(i)/(float)FFT_N * 2.0 * 3.14) * 32.0);
	}

	done = 1; 
	return;
*/

	note_current = note;
	note_divisor = notes[note].divisor;
	note_bar = notes[note].bar;

	capture_pos = 0;
	set_sleep_mode(SLEEP_MODE_ADC);
	ADCSRA |= (1<<ADSC);
//	while (capture_pos != FFT_N) sleep_mode();
	while (capture_pos != FFT_N);
}

void ADCInit(void)
{
	DDRA = 0x00;
	PORTA = 0x00;

	ADMUX = 2 | (1<<REFS0);

	/* Prescaler = / 128; 16*10^6 / 128 = 125000 */
	/* Prescaler = / 64; 16*10^6 / 64 = 250000 */
	/* / 13 cycles -> 19230.769230 Hz */
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0) | (1<<ADIE) | (1<<ADATE);

	/*
	 * 000  /2
	 * 001  /2
	 * 010  /4
	 * 011  /8
	 * 100  /16
	 * 101  /32
	 * 110  /64
	 * 111  /128
	 */

	ADCSRA |= (1<<ADEN);
}

void do_analysis(void) 
{
	uint16_t max = 0;
	uint16_t max_pos;

	uint16_t avg_sum;
	uint32_t avg;

	uint16_t avg_before_sum, avg_after_sum;
	uint32_t avg_before, avg_after;

	int i;
	uint16_t s;

	avg_before = avg_after = 0;
	avg_before_sum = avg_after_sum = 0;
	avg = avg_sum = 0;
	max = max_pos = 0;

	for (i = 5; i < FFT_N / 2 - 5; i++) {
		/* Filter out rubbish */
		s = (spektrum[i] /= 16);
		
		if (s < 10) 
			continue;

		/* Avg */
		avg += i * s;
		avg_sum += s;

		if (i < note_bar) {
			avg_before += s;
			avg_before_sum++;
		} else {
			avg_after += s;
			avg_after_sum++;
		}

		/* Max */
		if (s > max) {
			max = s;
			max_pos = i;
		}
	} 

	if (avg_sum)
		avg /= avg_sum;
	else
		avg = 0;

	if (avg_before_sum)
		avg_before /= avg_before_sum;
	else
		avg_before = 0;

	if (avg_after_sum)
		avg_after /= avg_after_sum;
	else
		avg_after = 0;

	uint16_t max_local;
	uint16_t max_local_pos;

	max_local = max_local_pos = 0;
	/* Find maximum nearest to our bar */
	for (i=0; i < 20; i++) {

		if (note_bar - i > 5) {
			s = spektrum[note_bar - i];
			if (s > max / 3 && s > max_local) {
				max_local = s;
				max_local_pos = -i;
			}
		}

		if (note_bar + i < FFT_N/2 - 5) {
			s = spektrum[note_bar + i];
			if (s > max / 3 && s > max_local) {
				max_local = s;
				max_local_pos = i;
			}
		}
	}

	printf("Max=%u Avg/B/A=%ld/%ld/%ld Local Max=%u\n", max_pos, avg, avg_before, avg_after, max_local_pos);
	
}
void GuitarIRTests(void)
{
	int i;
	uint16_t s;
	LEDInit();
	SerialInit();

	ADCInit();
	printf("Everything initialized\n");
	sei();

	/* Reference counter */
	TCCR0 = (1<<CS01) | (1<<WGM01); /* /8, CTC - clear timer on match */

	for (;;) {
		/* Wait for buffer to fill up */
		do_capture(NOTE_E);

		fft_input(capture, bfly_buff); 
		fft_execute(bfly_buff);
		fft_output(bfly_buff, spektrum);


/*		printf("Captured:\n");
		for (i = 0; i < FFT_N ; i++) {
			printf("%5d ", capture[i]);
			if ((i+1) % 20 == 0) {
				putchar('\n');
			}
		}
		putchar('\n');
		putchar('\n');
*/
		do_analysis();

		for (i = 5; i < FFT_N / 2 - 5; i++) {
			s = spektrum[i] / 16;

/*
			if (i % 3 == 2) {
				s = (spektrum[i] + spektrum[i-1] + spektrum[i-2]) / 3 / 16;
				printf("\n%3u: %5u  ", i, s);
				for (m = 0; m < s; m++) putchar('*');
			}
*/


/*
			if (i % 2 == 1) {
				s = (spektrum[i] + spektrum[i-1]) / 2 / 16;
				printf("\n%3u: %5u  ", i, s);
				for (m = 0; m < s; m++) putchar('*');
			}
*/



/*			printf("\n%3u: %5u  ", i, s);
			for (m = 0; m < s; m++) putchar('*');
*/
		}

		printf("Note=%d Bar=%d\n", note_current, note_bar);

/*		for (i=0; i<30; i++)
			_delay_ms(20);
*/

	}
}

int main(void) __attribute__((naked));
int main(void)
{
	int i;

	GuitarIRTests();

	DDRD |= (1<<PD6); // debug
	PORTD |= (1<<PD6);

	set_sleep_mode(SLEEP_MODE_IDLE);

	LEDInit();
	LEDOn(LED_R);

//	IRLedTest();

	SerialInit();
	printf("Serial initialized\n");

/*	LightInit();
	printf("Lights initialized\n");
*/

/*	Initialize();
	printf("Interrupts initialized \n");
*/
	LEDOff(LED_RGB);
	sei();
	

	for (;;) {
/*		sleep_mode();
		cli();
		i = Cl.Second;
		Cl = Clock;
		sei();
		if (i != Cl.Second) { // * Second changed *
			printf("%03d %02d:%02d.%02d [%03d]\n",
			       Cl.Day, Cl.Hour, Cl.Minute, Cl.Second, Cl.Part);
		} */
		for (i=0; i<50; i++) {
			PORTD ^= (1<<PD6); // debug
			_delay_ms(20);
		}
		printf("%u\n", i);
	}

	return 0;
}