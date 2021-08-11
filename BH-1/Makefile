FILE = bh1_r1_v2

all: $(FILE).hex
	avrdude -C /usr/local/Cellar/avrdude/6.3_1/etc/avrdude.conf -c usbtiny -p m328 -U flash:w:$(FILE).hex

$(FILE).hex: $(FILE).elf
	avr-objcopy -j .text -j .data -O ihex $(FILE).elf $(FILE).hex

$(FILE).elf: $(FILE).o
	avr-g++ -mmcu=atmega328 -o $(FILE).elf $(FILE).o

$(FILE).o: $(FILE).c
	avr-g++ -Os -mmcu=atmega328 -c $(FILE).c

clean:
	rm *.hex
	rm *.elf
	rm *.o
