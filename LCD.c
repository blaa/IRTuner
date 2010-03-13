/*
 * This program may be distributed under terms of GNU General Public License version 3.
 * (C) 2004 by Tomasz bla Fortuna
 *
 * NOTE: This functions have by default RW low. And RS high.
 */
#define LCDPort		PORTC	/* 4 pins port */
#define LCDDDR          DDRC

#define LCDCPort	PORTD 	/* Control pins port */
#define LCDCDDR		DDRD

#define LCD_RS		(1<<PD6)
#define LCD_RW		(1<<PD5)
#define LCD_E		(1<<PD4)

#define LCD_DB		0xF0	/* Mask for sending nibble to LCD; 0F - 0000 1111 
				   bits 0,1,2,3 of PORT are used. */
				/* 0x0F or 0xF0 is possible; */

#define lcd_display_on	1
#define lcd_display_off	0

/* Private */
uint8_t LCDx, LCDy;

/* Lowlevel function sending byte to LCD controller */
void lcd_send(const uint8_t byte)
{
	unsigned char nibble;
	LCDCPort &= ~LCD_RW;

	nibble	= ((byte & 0xF0) | (byte>>4)) & LCD_DB;		/* MSB */
	LCDPort	= (LCDPort & ~LCD_DB) | nibble;	/* Clear port with DB mask and OR with nibble. */

	usleep(1);						/* Send command */
	LCDCPort |= LCD_E;
	usleep(2);
	LCDCPort &= ~LCD_E;
	
	nibble	= ((byte & 0x0F) | (byte<<4)) & LCD_DB;		/* LSB */
	LCDPort	= (LCDPort & ~LCD_DB) | nibble;
	
	usleep(1);						/* Send command */
	LCDCPort |= LCD_E;
	usleep(2);
	LCDCPort &= ~LCD_E;
}

/* Public */
/* Function initializing LCD */
static inline void lcd_init()
{
	LCDCPort &= ~LCD_RW;			/* Write mode */
	LCDCPort &= ~LCD_RS;			/* Register - Instructions */
	LCDCPort &= ~LCD_E;

	LCDCDDR |= LCD_RW | LCD_E | LCD_RS;
	LCDDDR |= LCD_DB;

	/* 4bit mode? */
	const char nibble	= 0x22;
	LCDPort	= (LCDPort & ~LCD_DB) | nibble;	/* Clear port with DB mask and OR with nibble. */

	usleep(1);						/* Send command */
	LCDCPort |= LCD_E;
	usleep(2);
	LCDCPort &= ~LCD_E;
	
	lcd_send(0x2C);		/* Function Set: 4bit bus; 2 lines; 5x11 font */
	usleep(45);
	lcd_send(0x2C);		/* Function Set: 4bit bus; 2 lines; 5x11 font */
	usleep(45);
	lcd_send(0x0C);		/* Display Control: Display: ON; Cursor OFF; Blink OFF */
	usleep(45);
	lcd_send(0x01);		/* Clear display */
	msleep(2);		/* 1.53ms */
	lcd_send(0x06);		/* Entry Mode Set: Increment address; No shift */
	usleep(45);

	LCDx = 0; LCDy = 0;

	LCDCPort |= LCD_RS;
}

/* Goto XY location */
void lcd_goto(uint8_t x, uint8_t y)
{
	uint8_t Address;
	Address = (y*64+x) | 0x80;	/* Set DDRAM Address */
	LCDCPort &= ~LCD_RS;
	usleep(50);
	lcd_send(Address);
	usleep(45);
	LCDCPort |= LCD_RS;
	LCDx = x; LCDy = y;
}

/* Clears LCD and goes to first location on display */
void lcd_clear()
{
	LCDCPort &= ~LCD_RS;
	lcd_send(0x01);
	msleep(2);		/* 1.53ms */
	LCDCPort |= LCD_RS;
	LCDx = 0; LCDy = 0;
}

void lcd_display(uint8_t status)
{
	LCDCPort &= ~LCD_RS;
	if (status == lcd_display_on)
		lcd_send(0x0D);
	else
		lcd_send(0x09);
	usleep(45);
	LCDCPort |= LCD_RS;
}

void lcd_print(const char *string)
{
	const char *ch;
	for (ch = string; *ch; ch++)
	{
		if (*ch == '\n') {
			if (LCDy == 0) {
				LCDy = 1;
				LCDx = 0;
				lcd_goto(0, 1);
			}
		} else {
			if ((LCDx == 8) && (LCDy == 1)) break;
			/* if ((LCDx == 16) && (LCDy == 0)) lcd_goto(0,1); */
			lcd_send(*ch);
			LCDx++;
			usleep(45);
		}
	}
}


static inline void lcd_chars()
{
        LCDCPort &= (unsigned int) ~LCD_RS;
        lcd_send(0x40 + 8);                  /* Set CGRAM address to 0+8; Select CGRAM */
        usleep(45);

        LCDCPort |= LCD_RS;

        unsigned char byte = (1<<4);
        unsigned char i;
        unsigned char y;
        for (i=0; i<5; i++) {
	        for (y=0; y<8; y++) {
		        lcd_send(byte);
		        usleep(45);
	        }
	        byte >>= 1;
        }

        /* For center */
        byte = (1<<4);
        for (y=0; y<8; y++) {
	        lcd_send(byte);
	        byte ^= (1<<3);
	        usleep(45);
        }
        byte = 1;
        for (y=0; y<8; y++) {
	        lcd_send(byte);
	        byte ^= (1<<1);
	        usleep(45);
        }

        lcd_goto(LCDx,LCDy);             /* Return to DDRAM. */
}
