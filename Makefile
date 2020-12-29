MCU := atmega328p
PROGRAMMER := usbasp
MODULES := enc28j60/driver.o uart.o enc28j60/ip.c enc28j60/udp.c enc28j60/checksum.c
OUTPUT := main.elf
HEXOUT := main.ihex

GCC := /bin/avr-gcc
OBJCOPY := /bin/avr-objcopy
SIZE := /bin/avr-size
AVRDUDE := /bin/avrdude

GCC_ARGS += -mmcu=$(MCU)
GCC_ARGS += -Os
GCC_ARGS += -DF_CPU=16000000UL
GCC_ARGS += -Wall
GCC_ARGS += -Wextra

OBJCOPY_ARGS += -O ihex

SIZE_ARGS += --mcu=$(MCU)
SIZE_ARGS += --format=avr

AVRDUDE_ARGS += -p $(MCU)
AVRDUDE_ARGS += -P /dev/ttyACM0
AVRDUDE_ARGS += -c $(PROGRAMMER)

%.o: %.c
	$(GCC) $(GCC_ARGS) -c $< -o $@

all: $(MODULES)
	$(GCC) $(GCC_ARGS) main.c $(MODULES) -o $(OUTPUT)
hex:
	$(OBJCOPY) $(OBJCOPY_ARGS) $(OUTPUT) $(HEXOUT)
size:
	$(SIZE) $(SIZE_ARGS) $(OUTPUT)
avrdude:
	$(AVRDUDE) $(AVRDUDE_ARGS) -U flash:w:$(HEXOUT)
clean:
	rm -rf *.o
	rm -rf enc28j60/*.o