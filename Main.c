#define F_CPU 16000000UL

#include <stdio.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

/*** Local includes ***/
#include "LED.c"
#include "Serial.c"

/************************************************************************************************************
 * Guitar tunner - NOTES
 *
 * FFT_N = 256
 * FFT_N / 2 = max Hz
 * Musi wej¶æ FFT_N/4 pe³nych okresów danej czêstotliwo¶ci
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

/* num_t has 2 decimal places. */
typedef int32_t num_t;

const char spectrum_min = 10;
const char spectrum_max = FFT_N/2 - 10;
const int harm_max = 15;


union buffs {
	int16_t capture[FFT_N];      /* 512 bytes */
	uint16_t spectrum[FFT_N/2];  /* 256 bytes */
};
static union buffs A;

static union {
	complex_t bfly_buff[FFT_N];   /* 1024 bytes */
	struct {
		/*** Variables for analysis ***/
		uint16_t max;
		uint16_t max_pos;

		uint16_t avg_sum;
		uint32_t avg;

		uint16_t avg_before_sum, avg_after_sum;
		uint32_t avg_before, avg_after;

		/* We have to find all harmonics */
		num_t harms[15];
		uint16_t harm_wage[15];
		int harm_main; /* The one with biggest wage */
		uint16_t harm_main_wage;
		int harm_cnt;

		uint16_t running_avg;
		char dist_between_max;

		int times;
		int error;

		char num2str_buff[10];
	} v;
} B;

static volatile int capture_pos;               /* Position in buffer            */
static volatile int16_t adc_cur;               /* Current measurement           */
static volatile int16_t background;            /* Estimated Light background    */

/* Note to chase */
static volatile char note_current = 0;
static volatile char note_divisor = 30;
static volatile int note_bar = 30;

ISR(ADC_vect) 
{
	static char i;
	static int16_t tmp;

	/* Read measurement. It will get averaged */
	tmp = ADC - 512;

	/* Calculate background all the time */
	background *= 7;
	background += tmp;
	background /= 8;

	/* Increment divisor, return if too small */
	if (++i % note_divisor  != 0) {
		return;
	}

  	/* Average all measurements into one */
	adc_cur = tmp - background;

	/* Ignore saving if buffer is full */
	if (capture_pos >= FFT_N)
		return;

	/* Multiply to better fit FFT */
	adc_cur *= 500;

	/* Store */
	A.capture[capture_pos] = adc_cur;
	++capture_pos;
	
	/* After next measurement we're full! */
	if (capture_pos == FFT_N) {
		ADCSRA &= ~(1<<ADATE);
	}
}

enum { NOTE_E2 = 0, NOTE_A, NOTE_D, NOTE_G, NOTE_H, NOTE_E }; 
struct {
	char divisor;
	char bar;
	uint16_t freq;
} notes[] = {
	/* f=82.407 presc=128 div=30 bar=66 err=0.22521  */
	{30, 66, 8240U}, 
	/* f=110.000 presc=64 div=43 bar=63 err=0.05982  */
	{43, 63, 11000U}, 
	/* f=146.832 presc=128 div=17 bar=66 err=1.01045  */
	{17, 66, 14683U}, 
	/* f=195.998 presc=64 div=25 bar=65 err=0.68550  */
	{25, 65, 19599U}, 
	/* f=246.942 presc=128 div=9 bar=59 err=0.71470  */
	{9, 59, 24694U}, 
	/* f=329.628 presc=64 div=15 bar=66 err=0.90085  */
	{15, 66, 32962U}, 
};

const char *num2str(num_t number)
{
	int16_t rest = number % 100;
	if (rest < 0) rest = -rest;
	sprintf_P(B.v.num2str_buff, PSTR("%ld.%02d"), (int32_t)number / 100, rest);
	return B.v.num2str_buff;
}

/* bar should be multiplied by 10, e.g. 0 - 1280 */
static num_t bar2hz(const num_t bar)
{
	/* for prescaler /16: f = 75.1201923 * B / D */
	return ((7512UL * bar) / note_divisor) / 100;
}

static inline void do_capture(int note)
{
	note_current = note;
	note_divisor = notes[note].divisor;
	note_bar = notes[note].bar;

	capture_pos = 0;
	set_sleep_mode(SLEEP_MODE_ADC);
	ADCSRA |= (1<<ADSC) | (1<<ADATE);
	sei();

//	while (capture_pos != FFT_N) sleep_mode();
	while (capture_pos != FFT_N);
	cli();
}

static inline void ADCInit(void)
{
	DDRA = 0x00;
	PORTA = 0x00;

	ADMUX = 2 | (1<<REFS0);

	/* Prescaler = / 128; 16*10^6 / 128 = 125000 */
	/* Prescaler = / 64; 16*10^6 / 64 = 250000 */
	/* / 13 cycles -> 19230.769230 Hz */
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS1) | (1<<ADIE) | (1<<ADATE);

	/*
	 * 000  /2    001  /2
	 * 010  /4    011  /8
	 * 100  /16   101  /32
	 * 110  /64   111  /128
	 */

	ADCSRA |= (1<<ADEN);
}

/* Method: Calculating frequency */

static num_t estimate_bar(int16_t bar) 
{
	int16_t s;
	int32_t avg_local;
	uint16_t avg_local_sum;
	char i;

	avg_local = avg_local_sum = 0;
	for (i=1; i<=7; i++) {
		s = A.spectrum[bar + i - 4];
		avg_local += i * s;
		avg_local_sum += s;
	}
	avg_local *= 100;
	avg_local /= avg_local_sum;
	avg_local -= 400; /* 0 should be center (max_local_pos) */

	return bar*100 + avg_local;
}

static char is_good_max(const int16_t bar)
{
	/* 1) Pick is bigger than region average */
	uint32_t avg;
	char i;

	avg = 0;
	for (i=1; i<=7; i++) {
		avg += A.spectrum[bar + i - 4];
	}
	avg /= 7;

	if (avg + 5 > A.spectrum[bar])
		return 0;

	return 1;
}

static inline void spectrum_analyse(void) 
{
	/* Maximas:
	 * We should see our main freq at 64 bar it's harmonics: 32, 96
	 * We should see our main freq at note_bar it's harmonics: 
	 * note_bar-32, note_bar+96
	 */

	int i, m;
	uint16_t s;

	B.v.avg_before = B.v.avg_after = 0;
	B.v.avg_before_sum = B.v.avg_after_sum = 0;
	B.v.avg = B.v.avg_sum = 0;
	B.v.max = B.v.max_pos = 0;

	/* "Good sample" definition:
	 * Avg >= 10
	 * Max >= 20
	 *
	 *
	 *
	 */
	
	/* Precalculations:
	 * 1) Calculate global maximum for reference and it's position
	 * 2) Calculate average value for spectrum 
	 */
	for (i = spectrum_min; i < spectrum_max; i++) {
		/* Filter out rubbish */
		s = (A.spectrum[i] /= 16);
		
		if (s < 4) 
			continue;

		/* Avg */
		B.v.avg += s;
		B.v.avg_sum += 1;

		if (i < note_bar) {
			B.v.avg_before += s;
			B.v.avg_before_sum++;
		} else {
			B.v.avg_after += s;
			B.v.avg_after_sum++;
		}

		/* Max */
		if (s > B.v.max) {
			B.v.max = s;
			B.v.max_pos = i;
		}
	} 

	B.v.avg = B.v.avg_sum ? B.v.avg/B.v.avg_sum : 0;
	B.v.avg_before = B.v.avg_before_sum ? B.v.avg_before/B.v.avg_before_sum : 0;
	B.v.avg_after_sum = B.v.avg_after_sum ? B.v.avg_after/B.v.avg_after_sum : 0;

	printf_P(PSTR("AVG: %5ld -- %5ld -- %5ld\n"), B.v.avg_before, B.v.avg, B.v.avg_after);
	printf_P(PSTR("Max=%u at %u (f=%s)\n"), B.v.max, B.v.max_pos, num2str(bar2hz(B.v.max_pos*100)) );


	/* Calculate positions of all harmonics */
	B.v.harm_main_wage = 0;
	B.v.harm_main = -1;
	B.v.harm_cnt = 0;

	B.v.running_avg = 0;
	B.v.dist_between_max = 0;


	for (i = spectrum_min; i<spectrum_max; i++) {
		s = A.spectrum[i];

		if (B.v.dist_between_max) {
			B.v.dist_between_max--;
			goto not_max;
		}

		if (s <= B.v.running_avg + 2) 
			goto not_max;

		for (m=i-4; m <= i+4; m++) {
			if (s < A.spectrum[m])
				goto not_max;
		}

		if (s <= B.v.avg/4)
			goto not_max;

		if (is_good_max(i)) {
			const num_t real_bar = estimate_bar(i);
			const num_t freq = bar2hz(real_bar);
			printf_P(PSTR("MAX=%s "), num2str(real_bar));
			printf_P(PSTR("FREQ=%s\n"), num2str(freq));

			if (s > B.v.harm_main_wage) {
				/* Update main harmonic */
				B.v.harm_main_wage = s;
				B.v.harm_main = B.v.harm_cnt;
			}

			B.v.harms[B.v.harm_cnt++] = freq;
			B.v.harm_wage[B.v.harm_cnt] = s;

			if (B.v.harm_cnt == harm_max)
				break;

			/* Keep distance between maxes */
			B.v.dist_between_max = 4;
		}

	not_max:
		B.v.running_avg += s;
		B.v.running_avg /= 2;
	}


	uint32_t harm_avg_dist = 0;
	for (i=0; i<B.v.harm_cnt-1; i++) {
		// Average harmonics of the main harmonic 
		harm_avg_dist += B.v.harms[i+1] - B.v.harms[i];
	}
	harm_avg_dist /= B.v.harm_cnt - 1;
	printf_P(PSTR("Average distance between harmonics: %ld\n"), harm_avg_dist);

//		printf_P(PSTR("(main=%d) harm %d [%ld]: times %d, error: %d\n"), B.v.harm_main, i, B.v.harms[i], B.v.times, B.v.error);		
}

void spectrum_display(void)
{
	uint16_t s;
	int i, m;

	/* Horizontal spectrum: */
	for (i = 60; i>0; i-=3) {
		for (m = spectrum_min; m < spectrum_max; m++) {
			s = A.spectrum[m];
			if (s > i)
				putchar('*');
			else
				putchar(' ');
		}
		putchar('\n');
	}
	
	for (i = spectrum_min; i < spectrum_max; i++)
		if (i>=100)
			putchar('1');
		else 
			putchar(' ');
	putchar('\n');

	for (i = spectrum_min; i < spectrum_max; i++) {
		const char tmp  = (i % 100)/10;
		putchar(tmp + '0');
	}
	putchar('\n');

	for (i = spectrum_min; i < spectrum_max; i++)
		putchar(i % 10 + '0');
	putchar('\n');

	putchar('\n');

#if 0
	for (i = spectrum_min; i < spectrum_max; i++) {



/*
  if (i % 3 == 2) {
  s = (spectrum[i] + spectrum[i-1] + spectrum[i-2]) / 3;
  printf("\n%3u: %5u  ", i, s);
  for (m = 0; m < s; m++) putchar('*');
  }

*/

/*
  if (i % 2 == 1) {
  s = (spectrum[i] + spectrum[i-1]) / 2;
  printf("\n%3u: %5u  ", i, s);
  for (m = 0; m < s; m++) putchar('*');
  }
*/



/*			printf("\n%3u: %5u  ", i, s);
			for (m = 0; m < s; m++) putchar('*');
*/
	}
#endif

}

int main(void) __attribute__((naked));
int main(void)
{
	LEDInit();
	SerialInit();

	ADCInit();
	sei();

	printf_P(PSTR("Hidden variables size: %d\n"), sizeof(B.v));

	/* Reference counter */
	TCCR0 = (1<<CS01) | (1<<WGM01); /* /8, CTC - clear timer on match */
	LEDOn(LED_G);
	for (;;) {
		/* Wait for buffer to fill up */
		do_capture(NOTE_E);


		fft_input(A.capture, B.bfly_buff); 
		fft_execute(B.bfly_buff);
		fft_output(B.bfly_buff, A.spectrum);
		LEDOff(LED_G);

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

		printf_P(PSTR("\nNote=%d Bar=%d\n"), note_current, note_bar);
		spectrum_analyse();
		spectrum_display();



	}


	return 0;
}
