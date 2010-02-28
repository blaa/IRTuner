
#define LED_PORT	PORTB
#define LED_DDR		DDRB

#define LED_R		(1<<PB1)
#define LED_G		(1<<PB2)
#define LED_B		(1<<PB0)
#define LED_RGB		(LED_R|LED_G|LED_B)


static inline void LEDInit(void)
{
	LED_PORT &= ~(LED_R | LED_G | LED_B);
	LED_DDR |= LED_R | LED_G | LED_B;
}

static inline void LEDOn(unsigned char Mask)
{
	LED_PORT |= Mask & LED_RGB;
}

static inline void LEDOff(unsigned char Mask)
{
	LED_PORT &= ~(Mask & LED_RGB);
}

static inline void LEDSwitch(unsigned char Mask)
{
	LED_PORT ^= Mask & LED_RGB;
}
