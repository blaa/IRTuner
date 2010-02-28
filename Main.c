#define F_CPU 16000000UL
// #define inline inline __attribute__((always_inline))

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
 *
 */

#include "FFT/ffft.h"

/* FFT Buffers and data*/
#define var(x) x

/* num_t has 2 decimal places. */
typedef int32_t num_t;

const char spectrum_min = 10;
const char spectrum_max = FFT_N/2 - 10;
const int harm_max = 15;

int16_t capture[FFT_N];      /* 512 bytes */
uint16_t spectrum[FFT_N/2];  /* 256 bytes */


complex_t bfly_buff[FFT_N];   /* 1024 bytes */



static volatile int capture_pos;               /* Position in buffer            */
static volatile int16_t adc_cur;               /* Current measurement           */
static volatile int16_t background;            /* Estimated Light background    */

/* Note to chase */
volatile struct {
	char current;
	char divisor;
	int bar;
} note;

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
	if (++i % note.divisor != 0) {
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
	capture[capture_pos] = adc_cur;
	++capture_pos;
}

enum { NOTE_E2 = 0, NOTE_A, NOTE_D, NOTE_G, NOTE_H, NOTE_E }; 
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

const char *num2str(num_t number)
{
	static char num2str_buff[20];
	static int16_t rest;
	rest = number % 100;
	if (rest < 0) rest = -rest;
	sprintf_P(num2str_buff, PSTR("%ld.%02d"), (int32_t)(number / 100L), rest);
	return num2str_buff;
}

/* bar should be multiplied by 10, e.g. 0 - 1280 */
num_t bar2hz(const num_t bar)
{
	/* solve(16*10^6 / 64 / 13 / D / 2  *  (B/64) = f, f),numer;   */
	
	/* for prescaler / 128: f = 75.1201923 * B / D */
	return ((7512UL * bar) / note.divisor) / 100L;
}

static inline void do_capture(const int new_note)
{
	note.current = new_note;
	note.divisor = notes[new_note].divisor;
	note.bar = notes[new_note].bar;

	capture_pos = 0;

	


//	set_sleep_mode(SLEEP_MODE_ADC);
//	while (capture_pos != FFT_N) sleep_mode();
	while (capture_pos != FFT_N);
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

	ADCSRA |= (1<<ADEN) | (1<<ADSC);
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
		s = spectrum[bar + i - 4];
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
		avg += spectrum[bar + i - 4];
	}
	avg /= 7;

	if (avg + 5 > spectrum[bar])
		return 0;

	return 1;
}

static inline void spectrum_analyse(void) 
{
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




	/* Maximas:
	 * We should see our main freq at 64 bar it's harmonics: 32, 96
	 * We should see our main freq at note_bar it's harmonics: 
	 * note_bar-32, note_bar+96
	 */
	int i, m;
	uint16_t s;

	var(avg_before) = var(avg_after) = 0;
	var(avg_before_sum) = var(avg_after_sum) = 0;
	var(avg) = var(avg_sum) = 0;
	var(max) = var(max_pos) = 0;

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
		var(avg) += s;
		var(avg_sum) += 1;

		if (i < note.bar) {
			var(avg_before) += s;
			var(avg_before_sum)++;
		} else {
			var(avg_after) += s;
			var(avg_after_sum)++;
		}

		/* Max */
		if (s > var(max)) {
			var(max) = s;
			var(max_pos) = i;
		}
	} 


	var(avg) = var(avg_sum) ? var(avg)/var(avg_sum) : 0;
	var(avg_before) = var(avg_before_sum) ? var(avg_before)/var(avg_before_sum) : 0;
	var(avg_after_sum) = var(avg_after_sum) ? var(avg_after)/var(avg_after_sum) : 0;

/*
	printf_P(PSTR("AVG: %5ld -- %5ld -- %5ld\n"), var(avg_before), var(avg), var(avg_after));
	printf_P(PSTR("Max=%u at %u (f=%s)\n"), var(max), var(max_pos), num2str(bar2hz(var(max_pos)*100)) );
*/
	/* Calculate positions of all harmonics */
	var(harm_main_wage) = 0;
	var(harm_main) = -1;
	var(harm_cnt) = 0;

	var(running_avg) = 0;
	var(dist_between_max) = 0;

	for (i = spectrum_min; i<spectrum_max; i++) {
		s = spectrum[i];

		if (var(dist_between_max)) {
			var(dist_between_max)--;
			goto not_max;
		}

		if (s <= var(running_avg) + 2) 
			goto not_max;

		for (m=i-4; m <= i+4; m++) {
			if (s < spectrum[m])
				goto not_max;
		}

		if (s <= var(avg)/4)
			goto not_max;

		if (is_good_max(i)) {
			const num_t real_bar = estimate_bar(i);
			const num_t freq = bar2hz(real_bar);
			printf_P(PSTR("MAX=%s "), num2str(real_bar));
			printf_P(PSTR("FREQ=%s\n"), num2str(freq));

			if (s > var(harm_main_wage)) {
				/* Update main harmonic */
				var(harm_main_wage) = s;
				var(harm_main) = var(harm_cnt);
			}

			var(harms)[var(harm_cnt)++] = freq;
			var(harm_wage)[var(harm_cnt)] = s;

			if (var(harm_cnt) == harm_max)
				break;

			/* Keep distance between maxes */
			var(dist_between_max) = 4;

		}

	not_max:
		var(running_avg) += s;
		var(running_avg) /= 2;
	}



}

void spectrum_display(void)
{
	uint16_t s;
	int i, m;

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


static inline void memtest(void)
{
	int i;
	for (i=0; i<FFT_N; i++) {
		capture[i] = i;
	}
	for (i=0; i<FFT_N; i++) {
		if (capture[i] != i) {
			printf_P(PSTR("DUPA at %d\n"), i);
			LEDOn(LED_R);
			return;
		}
	}
	LEDOn(LED_G);
}

int main(void) __attribute__((naked));
int main(void)
{
	LEDInit();
	SerialInit();

	ADCInit();
	sei();

	/* Reference counter */
	// TCCR0 = (1<<CS01) | (1<<WGM01); /* /8, CTC - clear timer on match */
	puts_P(PSTR("Init"));
	for (;;) {
		/* Wait for buffer to fill up */
		do_capture(NOTE_A);
		puts_P(PSTR("Captured"));

		fft_input(capture, bfly_buff); 
		fft_execute(bfly_buff);
		fft_output(bfly_buff, spectrum);

		printf_P(PSTR("\nNote=%d Bar=%d Divisor=%d\n"), note.current, note.bar, note.divisor);
		spectrum_analyse();
		spectrum_display();
	}
	return 0;
}
