NAME = ps1-pad-memcard-iface

SRC = main.c

OBJ = $(SRC:.c=.o)
DEP = $(SRC:%.c=%.d)

# MCU type
MCU = atmega4809

# CPU frequency (per datasheet we can't use more than 10MHz @ 3.5V)
#
# I pick 4MHz so that I can easily divide it by 16 to get a 250MHz SPI clk.
F_CPU = 4000000L
# Frequency when divider is left by default
# F_CPU = 3333333L

# Programmer type
PROGRAMMER_TYPE = jtag2updi

# Programmer port
# PROGRAMMER_PORT = /dev/ttyACM0
PROGRAMMER_PORT = /dev/ttyUSB0

# AVR-GCC Flags
CFLAGS = -mmcu=$(MCU) -ffunction-sections -fdata-sections -DF_CPU=$(F_CPU) -Os -MMD -MP -flto -Wall

# PREFIX = /opt/avr8-gnu-toolchain-linux_x86_64/bin/

# Compilation command
CC = $(PREFIX)avr-gcc

# Build target
all: $(NAME).hex

# Compile
%.o: %.c
	$(info CC $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Link
$(NAME).elf: $(OBJ)
	$(info LD $@ $^)
	$(CC) $(CFLAGS) $^ -o $@

-include $(DEP)

# Hex file
$(NAME).hex: $(NAME).elf
	$(PREFIX)avr-objcopy -O ihex -R .eeprom $< $@

# Flash to board
#
# Arduino Nano Every default fuses:
#  fuse2/osccfg: 0x01 (16Mhz, set to 0x02 to run at 20Mhz)
#  fuse5/syscfg0: 0xc9
#  fuse8/bootend: 0x00
flash: all
	$(info FLASH $(NAME))
	stty -F $(PROGRAMMER_PORT) 1200
	sleep 1
	avrdude -p $(MCU) -c $(PROGRAMMER_TYPE) -P $(PROGRAMMER_PORT) -b115200 -e -D -Uflash:w:$(NAME).hex:i -Ufuse2:w:0x01:m

# Clean up
clean:
	$(info CLEAN $(NAME))
	rm -f $(NAME).elf $(NAME).hex $(OBJ) $(DEP)

.PHONY: all flash clean

# Be verbose if V is set
$V.SILENT:
