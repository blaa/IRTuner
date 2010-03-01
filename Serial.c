/***********************************
 * (C) 2008 by Tomasz bla Fortuna <bla@thera.be>.
 * License: GPL3+ (See Docs/LICENSE)
 *
 * stdio driven hardware UART for debugging.
 ********************/
const uint16_t UART_BAUDRATE = 16; /* 115200 */

static FILE Serial_stdout;

static int Serial_putchar(char c, FILE *Stream)
{
	if (c == '@')
		LEDOn(LED_G);

	while ( !(UCSR0A & (1<<UDRE0)) );
	UDR0 = c;

	if (c == '@')
		LEDOff(LED_G);

	return 0;
}

/*
static int Serial_getchar(FILE *Stream)
{
	if (!(UCSRA & (1<<RXC)))
		return -1;
	return UDR;
}
*/

static inline void SerialInit(void)
{
	/* Set baudrate */
	UBRR0H = (unsigned char)(UART_BAUDRATE>>8);
	UBRR0L = (unsigned char)UART_BAUDRATE;

	/* Double asynchronous UART speed */
	UCSR0A = (1<<U2X0);

	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);

	/* even parity, 8 bits of data, 1 stop bits */
	UCSR0C = /*(1<<URSEL0) | */(0<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01) | (1<<UPM01);

	DDRD |= (1<<PD1);
	PORTD |= (1<<PD1);

	fdev_setup_stream(&Serial_stdout, Serial_putchar, NULL, _FDEV_SETUP_RW);
	stdout = &Serial_stdout;
}

