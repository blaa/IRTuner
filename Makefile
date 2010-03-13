BIG=50000
OPT=-Os
LDFLAGS=-Wl,-gc-sections 
#-Wl,-u,vfprintf -lprintf_min

#OPT=-Os
CFLAGS=-I/usr/avr/include -pipe -mmcu=atmega32 $(OPT) $(LDFLAGS) -Wall -Winline $(INLINE)
CC=avr-gcc
UISP=uisp

Main: Main.c Serial.c FFT/ffft.S Sleep.c LCD.c
	$(CC) $(CFLAGS) -o Main Main.c FFT/ffft.S
	$(CC) -S $(CFLAGS) -o Main.s Main.c > /dev/null 2>&1
	avr-objcopy -j .text -j .data -O ihex Main Main.hex
	avr-objcopy -j .text -j .data -O binary Main Main.binary
	avr-objcopy -j .eeprom -O ihex Main Main.eeprom
#-Wl,-u,vfprintf -lprintf_min

.PHONY: Send SendN Fuses EEPROM

Send: Main
	# $(UISP) -dlpt=/dev/parport0 --segment=flash --erase -dprog=dapa --upload if=Main.hex -dpart=atmega32 --verify
	# avrdude -c usbasp -p m32 -U flash:w:./Main.hex:i   -v 
	avrdude -c usbasp -p m32 -U flash:w:./Main.hex:i   -v -F

SendN: Main
	$(UISP) -dlpt=/dev/parport0 --segment=flash --erase -dprog=dapa --upload if=Main.hex -dpart=atmega64 

Memory: Main
	avr-objdump -h Main

Fuses:
	$(UISP) -dlpt=/dev/parport0 -dprog=dapa --rd_fuses -dpart=atmega64 

EEPROM:
	$(UISP) -dlpt=0x378 -dprog=dapa --segment=eeprom --download of=EEPROM.srec -dpart=atmega32 
	cat EEPROM.srec
	../srec_to_bin <  EEPROM.srec > EEPROM.binary

clean:
	rm -f Main.hex Main Main.s *.o Main.binary Main.eeprom
