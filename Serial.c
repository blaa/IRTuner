/***********************************
 * (C) 2008 by Tomasz bla Fortuna <bla@thera.be>.
 * License: GPL3+ (See LICENSE)
 *
 * stdio driven hardware UART for debugging.
 ********************/
const uint16_t UART_BAUDRATE = 16; /* 115200 */

static FILE serial_stdout;

static int serial_putchar(char c, FILE *Stream)
{
	while ( !(UCSRA & (1<<UDRE)) );
	UDR = c;
	return 0;
}

/*
static int serial_getchar(FILE *Stream)
{
	if (!(UCSRA & (1<<RXC)))
		return -1;
	return UDR;
}
*/

static inline void serial_init(void)
{
	/* Set baudrate */
	UBRRH = (unsigned char)(UART_BAUDRATE>>8);
	UBRRL = (unsigned char)UART_BAUDRATE;

	/* Double asynchronous UART speed */
	UCSRA = (1<<U2X);

	/* Enable receiver and transmitter */
	UCSRB = (1<<RXEN) | (1<<TXEN);

	/* even parity, 8 bits of data, 1 stop bits */
	UCSRC = (1<<URSEL) | (0<<USBS) | (1<<UCSZ0) | (1<<UCSZ1) | (1<<UPM1);

	DDRD |= (1<<PD1);
	PORTD |= (1<<PD1);

	fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_RW);
	stdout = &serial_stdout;
}

