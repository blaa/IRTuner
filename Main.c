#define F_CPU 16000000UL
#define inline

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/*** Local includes ***/
#include "LED.c"
#include "Serial.c"

static void error(void)
{
	static int i;  
	static int cnt;
	LEDOff(LED_RGB);
	cnt = 5;
	do {
		LEDSwitch(LED_R);
		for (i=0; i<20; i++)
			_delay_ms(20);
	} while (--cnt);
}

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
 *
 */

#include "FFT/ffft.h"

/* Strings */
#define _(x) x
#define printf printf

/* FFT Buffers and data*/
#define v(x) v.vars.x

/* num_t has 2 decimal places. */
typedef int32_t num_t;

/*** Constants ***/
const char spectrum_min = 10;
const char spectrum_max = FFT_N/2 - 10;
const int harm_max = 15;

/*** Buffers + Variables ***/
union {
	/* Buffer we store captured data in
	 * inside FFT is calculated and then transposed
	 * into spectrum */
	complex_t fft_buff[FFT_N];   /* 512 bytes */

	/* After fft_buff is unused we can use it's memory
	 * to hold variables required during analysis */
	struct {
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
	} vars;
} v;

/* Final version of spectrum for analysis */
uint16_t spectrum[FFT_N/2];  /* 128 bytes */

/* Buffer traversing for ADC interrupt */
volatile const prog_int16_t *window_cur = tbl_window;
const complex_t *fft_buff_end = &v.fft_buff[FFT_N];
volatile complex_t * volatile fft_buff_cur = &v.fft_buff[FFT_N];

/* Incremented in ADC with 16*10^6/ 128 / 13 = 9615 Hz freq */
volatile uint16_t tick;

/*** NOTE data ***/

/* Current selected note (divisor is required 
 * for gathering windowed data) */
volatile struct {
	char current;
	char divisor;
	int bar;
} note;

enum { NOTE_E2 = 0, NOTE_A, NOTE_D, NOTE_G, NOTE_B, NOTE_E };
struct {
	char divisor;
	char bar;
	uint16_t freq;
} notes[] = {
	/* f=82.407 presc=128 div=29 bar=32 err=0.48425 scale=64  */
	{29, 32, 8240U},
	/* f=110.000 presc=128 div=22 bar=32 err=0.73427 scale=64  */
	{22, 32, 11000U},
	/* f=146.832 presc=128 div=16 bar=31 err=1.28663 scale=64  */
	{16, 31, 14683U},
	/* f=195.998 presc=128 div=12 bar=31 err=1.93750 scale=64  */
	{12, 31, 19599U},
	/* f=246.942 presc=128 div=10 bar=33 err=0.95463 scale=64  */
	{10, 33, 24694U},
	/* f=329.628 presc=128 div=7 bar=31 err=3.04714 scale=64  */
	{7, 31, 32962U},
};


/* Initialize data for capture, select tone */
static inline void do_capture(const int new_note)
{
	note.current = new_note;
	note.divisor = notes[new_note].divisor;
	note.bar = notes[new_note].bar;

	window_cur = tbl_window;
	fft_buff_cur = v.fft_buff;

	set_sleep_mode(SLEEP_MODE_ADC);
  //	while (fft_buff_cur != fft_buff_end) sleep_mode();
	while (fft_buff_cur != fft_buff_end);
}

/* Reads data from ADC */
ISR(ADC_vect)
{
	/* Current measurement */
	static int16_t adc_cur;

	/* Estimated Light background */
	static int16_t background;

	/* Used for additional dropping of incomming data */
	static char i;

	/* Read measurement. It will get averaged */
	adc_cur = ADC - 512;

	tick++;

	/* Calculate background all the time */
	background *= 7;
	background += adc_cur;
	background /= 8;

	/* Ignore saving if buffer is full */
	if (fft_buff_cur == fft_buff_end)
		return;

	/* Increment divisor and drop some results */
	if (++i % note.divisor != 0)
		return;

	/* Remove background and multiply to better fit FFT algorithm */
	adc_cur -= background;
	adc_cur *= 500;

	/* Store */
	const int16_t tmp = fmuls_f(adc_cur, pgm_read_word_near(window_cur));
	fft_buff_cur->r = fft_buff_cur->i = tmp;

	/* Increment buffers */
	fft_buff_cur++;
	window_cur++;
}


/* DEBUG function. Converts num_t into a string */
static const char *num2str(num_t number)
{
	static char num2str_buff[20] = {0};
	static int16_t rest;
	rest = number % 100;
	if (rest < 0) rest = -rest;

	itoa((int16_t)(number/100L), num2str_buff, 10);
	const int pos = strlen(num2str_buff);
	num2str_buff[pos] = '.';
	itoa(rest, num2str_buff+pos+1, 10);

	return num2str_buff;
}

/* Convert accurate bar position (two decimal places) 
 * into frequency according to current note divisor */
static num_t bar2hz(const num_t bar)
{
	/* solve(16*10^6 / 64 / 13 / D / 2  *  (B/64) = f, f),numer;   */

	/* for prescaler / 128: f = 75.1201923 * B / D */
	return ((7512UL * bar) / note.divisor) / 100L;
}

/* Method: Calculating frequency */
static num_t estimate_bar(const int16_t bar)
{
	int32_t s;
	int32_t avg;
	int32_t avg_sum;
	int i;

	avg = avg_sum = 0;
	for (i=1; i<=7; i++) {
		s = spectrum[bar + i - 4];
		avg += i * s;
		avg_sum += s;
	}
	avg *= 100;
	avg /= avg_sum;
	avg -= 400;
	avg_sum /= 7; /* Calculate neighborhood average */

	if (avg_sum + 5 > spectrum[bar]) 
		return 0;
	else
		return (num_t)bar*100L + avg;
}

static inline void spectrum_analyse(void)
{
	/* Maximas:
	 * We should see our main freq at 64 bar it's harmonics: 32, 96
	 * We should see our main freq at note_bar it's harmonics:
	 * note_bar-32, note_bar+96
	 */
	static int i, m;
	static uint16_t s;

	v(avg_before) = v(avg_after) = 0;
	v(avg_before_sum) = v(avg_after_sum) = 0;
	v(avg) = v(avg_sum) = 0;
	v(max) = v(max_pos) = 0;

	/* Precalculations:
	 * 1) Calculate global maximum for reference and it's position
	 * 2) Calculate average value for spectrum
	 */
	for (i = spectrum_min; i < spectrum_max; i++) {
		/* Filter out rubbish */
		s = (spectrum[i] /= 16);

		if (s < 4)
			continue;

		/* Avg */
		v(avg) += s;
		v(avg_sum) += 1;

		if (i < note.bar) {
			v(avg_before) += s;
			v(avg_before_sum)++;
		} else {
			v(avg_after) += s;
			v(avg_after_sum)++;
		}

		/* Max */
		if (s > v(max)) {
			v(max) = s;
			v(max_pos) = i;
		}
	}

	v(avg) = v(avg_sum) ? v(avg)/v(avg_sum) : 0;
	v(avg_before) = v(avg_before_sum) ? v(avg_before)/v(avg_before_sum) : 0;
	v(avg_after_sum) = v(avg_after_sum) ? v(avg_after)/v(avg_after_sum) : 0;


	printf(_("AVG: %5ld -- %5ld -- %5ld\n"), v(avg_before), v(avg), v(avg_after));
	printf(_("Max=%u at %u (f=%s)\n"), v(max), v(max_pos), num2str(bar2hz(v(max_pos)*100)) );

	/* Calculate positions of all harmonics */
	v(harm_main_wage) = 0;
	v(harm_main) = -1;
	v(harm_cnt) = 0;

	v(running_avg) = 0;
	v(dist_between_max) = 0;

	for (i = spectrum_min; i<spectrum_max; i++) {
		s = spectrum[i];

		if (v(dist_between_max)) {
			v(dist_between_max)--;
			goto not_max;
		}

		if (s <= v(running_avg) + 2)
			goto not_max;

		for (m=i-4; m <= i+4; m++) {
			if (s < spectrum[m])
				goto not_max;
		}

		if (s <= v(avg)/4)
			goto not_max;

		const num_t real_bar = estimate_bar(i);
		if (real_bar != 0) {
			const num_t freq = bar2hz(real_bar);
			printf(_("Loc_simp=%d MAX_realpos=%s "), i, num2str(real_bar));
			printf(_("FREQ=%s\n"), num2str(freq));

			if (s > v(harm_main_wage)) {
				/* Update main harmonic */
				v(harm_main_wage) = s;
				v(harm_main) = v(harm_cnt);
			}

			v(harms)[v(harm_cnt)++] = freq;
			v(harm_wage)[v(harm_cnt)] = s;

			if (v(harm_cnt) == harm_max)
				break;

			/* Keep distance between maxes */
			v(dist_between_max) = 4;
		}

	not_max:
		v(running_avg) += s;
		v(running_avg) /= 2;
	}
}

static void spectrum_display(void)
{
	static uint16_t s;
	static int i, m;

	/* Horizontal spectrum: */
	for (i = 60; i>0; i-=3) {
		for (m = spectrum_min; m < spectrum_max; m++) {
			s = spectrum[m];
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
}

static void self_test(void)
{
	/* Check input from ADC. It should 
	 * have some sensible values */
	int count;
	int drop;
	uint16_t tmp, min, max;

	/* Initialize some memory */
	for (count = 0; count < FFT_N; count++) {
		v.fft_buff[count].i = count;
		v.fft_buff[count].r = 32000 - count;
	}

	while (tick < 100);
	cli();

	for (;;) {
		count = 100;
		min=32768;
		max=0;
		do {
			drop = 100;
			do {
				while( !(ADCSRA & (1<<ADIF)) );
				ADCSRA |= (1<<ADIF);
			} while (--drop);
			tmp = ADC;
			if (tmp < min)
				min = tmp;
			if (tmp > max)
				max = tmp;
		} while(--count);
		
		if (max < 10 || min > 800)
			error();
		else if (max == min)
			error();
		else {
			LEDOn(LED_G);
			break;
		}
	}

	/* Check if memory still holds it's values */
	for (count=0; count < FFT_N; count++) {
		if (v.fft_buff[count].i != count) {
			for (;;) error();
		}
		if (v.fft_buff[count].r != 32000 - count) {
			for (;;) error();
		}
	}

        sei();
}

inline void ADCInit(void)
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
	ADCSRA |= (1<<ADEN) | (1<<ADSC);
}

int main(void)
{
	LEDInit();
	LEDOn(LED_B);

	SerialInit();
	ADCInit();

	sei();

	self_test();

	printf("Init\n");

        LEDOff(LED_RGB);
	for (;;) {
		/* Wait for buffer to fill up */
		do_capture(NOTE_E2);

//		fft_input(buff(capture), v.fft_buff);
		fft_execute(v.fft_buff);
		fft_output(v.fft_buff, spectrum);

		printf(_("\nNote=%d Bar=%d Divisor=%d\n"), note.current, note.bar, note.divisor);
		spectrum_analyse();
		spectrum_display();
	}
	return 0;
}
