FILE = dt2_v1

all: $(FILE).hex
	avrdude -C /usr/local/Cellar/avrdude/6.3_1/etc/avrdude.conf -c usbtiny -p m328p -U flash:w:$(FILE).hex

$(FILE).hex: $(FILE).elf
	avr-objcopy -j .text -j .data -O ihex $(FILE).elf $(FILE).hex

$(FILE).elf: $(FILE).o
	avr-gcc -mmcu=atmega328 -o $(FILE).elf $(FILE).o

$(FILE).o: $(FILE).c
	avr-gcc -Os -mmcu=atmega328 -c $(FILE).c

clean:
	rm *.hex
	rm *.elf
	rm *.o
