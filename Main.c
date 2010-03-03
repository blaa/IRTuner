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
#include "Sleep.c"
#include "LCD.c"
#include "Serial.c"

static void error(void)
{
	static int i;
	static int cnt;
	cnt = 5;
	lcd_clear();

	lcd_print("ERROR\nVery bad.");

	for (i=0; i<3; i++)
		_delay_ms(20);
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
 *  Measurements for data:
	// f=82.407 presc=128 div=29 bar=32 err=0.48425 scale=64 
	{29, 8240U, 3},
	// f=110.000 presc=128 div=22 bar=32 err=0.73427 scale=64 
	{22, 11000U, 3},
	// f=146.832 presc=128 div=16 bar=31 err=1.28663 scale=64 
	{16, 14683U, 5},
	// f=195.998 presc=128 div=12 bar=31 err=1.93750 scale=64 
	{12, 19599U, 7},
	// f=246.942 presc=128 div=10 bar=33 err=0.95463 scale=64 
	{10, 24694U, 8},
	// f=329.628 presc=128 div=7 bar=31 err=3.04714 scale=64  
	{7, 32962U, 10},

 * NOTE    Difference
 * E2.        OK
 * A.        116.94 ~ +7Hz
 * D.         OK
 * G         198.6  ~ +4
 * B         253.7  ~ +7
 * E         324    ~ -5
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
const int harm_max = 4;

/*** Buffers + Variables ***/
union {
	/* Buffer we store captured data in
	 * inside FFT is calculated and then transposed
	 * into spectrum */
	complex_t fft_buff[FFT_N];   /* 512 bytes */

	/* After fft_buff is unused we can use it's memory
	 * to hold variables required during analysis */
	struct {
		/* Generic helper for calculating averages */
		uint16_t avg_helper;

		/* For locating maximas */
		uint32_t avg_global;

		uint16_t running_avg;
		char dist_between_max;

		/* Number of harmonics found */
		int harm_cnt;

		/* Index of the one with biggest wage + it's wage */
		int harm_main;
		uint16_t harm_main_wage;

		/* Harmonics found + their wages */
		num_t harm_freq[4];
		int16_t harm_bar[4];
		uint16_t harm_wage[4];

		/* Averaging correct frequency */
		num_t avg_freq;

		char lcd_buff[20];
	} vars;
} v;

/* Final version of spectrum for analysis */
uint16_t spectrum[FFT_N/2];  /* 128 bytes */

/* Buffer traversing for ADC interrupt */
volatile const prog_int16_t *window_cur = tbl_window;
const complex_t *fft_buff_end = &v.fft_buff[FFT_N];
volatile complex_t * volatile fft_buff_cur = &v.fft_buff[FFT_N];


/* Resulting frequency is averaged further */
num_t avg_freq_running;

/* And ignored after some time of no measurements */
uint16_t avg_freq_running_time;


/* Incremented in ADC with 16*10^6/ 128 / 13 = 9615 Hz freq */
volatile uint16_t tick;

/*** NOTE data ***/

/* Current selected note (divisor is required
 * for gathering windowed data) */
volatile struct {
	char current;
	char divisor;
	num_t freq;
	int time_relevant;
	int correction;
} note;

enum { NOTE_E2 = 0, NOTE_A, NOTE_D, NOTE_G, NOTE_B, NOTE_E };
struct {
	char divisor;
	uint16_t freq;
	int time_relevant; /* Time in which running average of freq is relevant */
	int correction;
} notes[] = {
	/* f=82.407 presc=128 div=29 bar=32 err=0.48425 scale=64  */
	{29, 8240U, 3, 0},
	/* f=110.000 presc=128 div=22 bar=32 err=0.73427 scale=64  */
	{22, 11000U, 3, -650},
	/* f=146.832 presc=128 div=16 bar=31 err=1.28663 scale=64  */
	{16, 14683U, 5, 0},
	/* f=195.998 presc=128 div=12 bar=31 err=1.93750 scale=64  */
	{12, 19599U, 7, -350},
	/* f=246.942 presc=128 div=10 bar=33 err=0.95463 scale=64  */
	{10, 24694U, 8, -750},
	/* f=329.628 presc=128 div=7 bar=31 err=3.04714 scale=64  */
	{7, 32962U, 10, 500},
};


void lcd_update(void)
{
	/* First line - accuracy of current measurement */

	/* Second: X ABC.DE - frequency + Current note */
	lcd_clear();

	lcd_print("line\n");

/*
	static char num2str_buff[20] = {0};
	static int16_t rest;
	rest = number % 100;
	if (rest < 0) rest = -rest;

	itoa((int16_t)(number/100L), num2str_buff, 10);
	const int pos = strlen(num2str_buff);
	num2str_buff[pos] = '.';
	itoa(rest, num2str_buff+pos+1, 10);

	return num2str_buff;


	sprintf(v(lcd_buff), "
*/


}



/* Initialize data for capture, select tone */
static inline void do_capture(const int new_note)
{
	note.current = new_note;
	note.divisor = notes[new_note].divisor;
	note.freq = (num_t) notes[new_note].freq;
	note.time_relevant = notes[new_note].time_relevant;
	note.correction = notes[new_note].correction;

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
	adc_cur *= 1000;

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

#define int2num(x) (100L*x)

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
	int32_t avg;
	int32_t avg_sum;
	int i;

	avg = avg_sum = 0;
	for (i=1; i<=7; i++) {
		const int32_t s = spectrum[bar + i - 4];
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
	int i, m;
	uint16_t s;

	v(avg_global) = v(avg_helper) = 0;

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
		v(avg_global) += s;
		v(avg_helper) += 1;
	}

	v(avg_global) = v(avg_helper) ? v(avg_global)/v(avg_helper) : 0;

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

		if (s <= v(avg_global))
			goto not_max;

		const num_t real_bar = estimate_bar(i);
		if (real_bar != 0) {
			const num_t freq = bar2hz(real_bar);
			printf(_("Bar=%d / %s "), i, num2str(real_bar));
			printf(_("FREQ=%s Value=%u (avg=%u)\n"), num2str(freq), spectrum[i], v(avg_global));

			if (s > v(harm_main_wage)) {
				/* Update main harmonic */
				v(harm_main_wage) = s;
				v(harm_main) = v(harm_cnt);
			}

			v(harm_freq)[v(harm_cnt)] = freq;
			v(harm_bar)[v(harm_cnt)] = i;
			v(harm_wage)[v(harm_cnt)] = s;

			v(harm_cnt)++;

			if (v(harm_cnt) == harm_max)
				break;

			/* Keep distance between maxes */
			v(dist_between_max) = 4;
		}

	not_max:
		v(running_avg) += s;
		v(running_avg) /= 2;
	}

	/* Count time for running freq so we will forget it after while */
	if (avg_freq_running_time)
		avg_freq_running_time--;

	/* if there're 3 harmonics visible */
	switch (v(harm_cnt)) {
	case 3:
		/* Do the average of all of them treating them sequentially */
		v(avg_freq) = v(harm_freq)[1];
		v(avg_freq) += v(harm_freq)[0] * 2;
		v(avg_freq) += v(harm_freq)[2] * 2 / 3;
		v(avg_freq) /= 3;
		break;

	case 2:
		if (v(harm_bar)[0] < 25) {
			v(avg_freq) = v(harm_freq)[0] * 2;
			if (v(harm_bar)[1] < 38)
				v(avg_freq) += v(harm_freq)[1];
			else
				v(avg_freq) += v(harm_freq)[1] * 2 / 3;
		} else if (v(harm_bar)[0] < 38) {
			v(avg_freq) = v(harm_freq)[0];
			v(avg_freq) += v(harm_freq)[1] * 2 / 3;
		} else {
			/* Ok, something is wrong! */
			printf("ERR:Something wrong (%d)\n", v(harm_bar)[0]);
			return;
		}

		v(avg_freq) /= 2;
		break;
	default:
		return;
	}


	v(avg_freq) += note.correction;

	if (avg_freq_running_time) {
		avg_freq_running += v(avg_freq);
		avg_freq_running /= 2;
	} else {
		avg_freq_running = v(avg_freq);
	}

	avg_freq_running_time = note.time_relevant;

	printf("FREQUENCY         =%s\n", num2str(v(avg_freq)));
	printf("RUNNING FREQUENCY =%s\n", num2str(avg_freq_running));

	lcd_clear();
	if (note.freq < avg_freq_running - int2num(1)) {
		printf("TOO HIGH\n");
		lcd_print("too high\n");
	} else if (note.freq > avg_freq_running + int2num(1)) {
		printf("TOO LOW\n");
		lcd_print("too high\n");
	} else {
		printf("TUNED\n");
		lcd_print(" TUNED! ");
	}

}

static void spectrum_display(void)
{
	static uint16_t s;
	static int i, m;
	const int wider = 10;

	/* Horizontal spectrum: */
	for (i = 60; i>0; i-=3) {
		for (m = spectrum_min-wider; m < spectrum_max+wider; m++) {
			s = spectrum[m];
			if (s > i)
				putchar('*');
			else
				putchar(' ');
		}
		putchar('\n');
	}

	for (i = spectrum_min-wider; i < spectrum_max+wider; i++)
		if (i>=100)
			putchar('1');
		else
			putchar(' ');
	putchar('\n');

	for (i = spectrum_min-wider; i < spectrum_max+wider; i++) {
		const char tmp  = (i % 100)/10;
		putchar(tmp + '0');
	}
	putchar('\n');

	for (i = spectrum_min-wider; i < spectrum_max+wider; i++)
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

			PORTA ^= (1<<PA1); /* Blink IR light */

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
			break;
		}
	}

	PORTA |= (1<<PA1); /* Disable IR light */

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

//	ADMUX = 0 | (1<<REFS0);
	ADMUX = 3 | (1<<REFS0);

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
	SerialInit();
	ADCInit();

	_delay_ms(20);
	_delay_ms(20);
	_delay_ms(20);

	lcd_init();
	lcd_chars();
	lcd_clear();
	lcd_print("LCDInit\n");

	sei();

	/* Enable output IR */
	PORTA |= (1<<PA1);
	DDRA |= (1<<PA1);

	self_test();

	/* Enable IR */
	PORTA &= ~(1<<PA1);

	lcd_print("Init OK");

	for (;;) {
		/* Wait for buffer to fill up */
		do_capture(NOTE_E);

		fft_execute(v.fft_buff);
		fft_output(v.fft_buff, spectrum);

		printf(_("\nNote=%d freq=%s Divisor=%d\n"), note.current, num2str(note.freq), note.divisor);
		spectrum_analyse();
		spectrum_display();
	}
	return 0;
}
