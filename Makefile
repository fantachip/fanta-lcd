CC = avr-gcc -mmcu=atmega8 -Wall -Os
CCFLAGS = -Ofast -s -DNDEBUG -std=c99 -ffunction-sections -fno-exceptions -I/usr/local/avr/include/ 
LDFLAGS = -lprintf_flt -lm -L/usr/local/avr/lib/avr4/ 
AVRDUDE = avrdude -p m8 -c usbasp -e

%.o: %.c *.h
	$(CC) -Wa,-ahl=$<.s $(CCFLAGS) $< -c
	
all:
	make fantalcd.elf
	
fantalcd.elf: fanta-lcd.o 
	$(CC) $^ -o $@ $(LDFLAGS)
	avr-objcopy -j .text -j .data -O ihex fantalcd.elf fantalcd.hex
																																																										
install:
	$(AVRDUDE) -F -U flash:w:fantalcd.hex

# Fuse high byte:
# 0xc9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (full output swing)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ WDTON (WDT not always on)
#        +-------------------- RSTDISBL (reset pin is enabled)
# Fuse low byte:
# 0x9f = 1 0 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ BODEN (BrownOut Detector enabled)
#        +-------------------- DIV8 
fuse:
	#$(AVRDUDE) -U hfuse:w:0xc9:m -U lfuse:w:0x9f:m
	#factory settings
	$(AVRDUDE) -U hfuse:w:0xc9:m -U lfuse:w:0x91:m 
clean:
	rm -rf *.o *.elf *.hex
	#cp /usr/local/avr/lib/avr4/crtm8.o .
