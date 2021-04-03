TARGET = main

# Set the chip architecture and path to the linker script
LD_SCRIPT = linker_script.ld
CPU_TARGET = cortex-m0

# Define the toolchains to be used for compilation
TOOLCHAIN = /usr
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
AS = $(TOOLCHAIN)/bin/arm-none-eabi-as
LD = $(TOOLCHAIN)/bin/arm-none-eabi-ld
OC = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
OD = $(TOOLCHAIN)/bin/arm-none-eabi-objdump
OS = $(TOOLCHAIN)/bin/arm-none-eabi-size

# Assembly directives
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(CPU_TARGET)
ASFLAGS += -mthumb
ASFLAGS += -Wall

# C compiler directives
CFLAGS += -mcpu=$(CPU_TARGET)
CFLAGS += -mthumb
CFLAGS += -Wall
CFLAGS += -g3
CFLAGS += -O

# Linker directives
LSCRIPT = ./$(LD_SCRIPT)
LFLAGS += -mcpu=$(CPU_TARGET)
LFLAGS += -mthumb
LFLAGS += -Wall
LFLAGS += -nostdlib
LFLAGS += -lgcc
LFLAGS += -T$(LSCRIPT)

STARTUP = ./startup.s
C_SRC = ./main.c
C_SRC += ./peripherals.c
C_SRC += ./ws2812.c

OBJS = $(STARTUP:.s=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

flash: $(ELF_DEBUG)
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
		-c "program $(TARGET).elf verify reset exit"

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
