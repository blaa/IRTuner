#include <util/delay.h>

#define msleep(x) _delay_ms(x)
#define usleep(x) _delay_us(x)

void sleep(char count)
{ 
	msleep(250 * count);
	msleep(250 * count);
	msleep(250 * count);
	msleep(250 * count);
}

