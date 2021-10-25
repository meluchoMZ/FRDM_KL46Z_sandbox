# Embedded systems, 2021
# Miguel Blanco Godón

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
CFLAGS=-DCPU_MKL46Z256VLL4 -I./include/ -I./src/include/ -g -Wall
LDFLAGS= -g --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
TARGET=led_blinky
SRC=$(shell find . -type f -iname '*.c')
HEADERS=$(shell find . -type f -iname '*.h')
OBJ=build/*.o

all: $(TARGET).elf

$(OBJ): $(SRC)
	@echo Compiling files: $(SRC)
	$(CC) -c $(ARCHFLAGS) $(CFLAGS)  $(SRC)
	mv *.o build/

$(TARGET).elf: $(OBJ)
	@echo Linking files: $(OBJ)
	$(LD) $(LDFLAGS) -o $@ $(OBJ)

binary: $(TARGET).elf
	@echo Creating binary file $(TARGET).bin
	arm-none-eabi-objcopy --output-target=binary $< $(TARGET).bin

clean:
	find . -name '*.o' -type f -delete
	rm $(TARGET).elf $(TARGET).map
