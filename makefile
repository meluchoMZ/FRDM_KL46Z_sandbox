# Embedded systems, 2021
# Miguel Blanco Godón

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
CFLAGS=-DCPU_MKL46Z256VLL4 -I./include/ -I./src/include/ -g -Wall -ffunction-sections
LDFLAGS= -g --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld -ffunction-sections
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
TARGET=main
SRC=$(shell find . -type f -iname '*.c')
HEADERS=$(shell find . -type f -iname '*.h')
OBJ=build/*.o

all: $(TARGET).elf

$(OBJ): $(SRC)
	@echo Compiling files: $(SRC)
	@echo "" 
	$(CC) -c $(ARCHFLAGS) $(CFLAGS)  $(SRC)
	mv *.o build/
	@echo "" 

$(TARGET).elf: $(OBJ)
	@echo Linking files: $(OBJ)
	@echo "" 
	$(LD) $(LDFLAGS) -o $@ $(OBJ)
	@echo "" 

binary: $(TARGET).elf
	@echo Creating binary file $(TARGET).bin
	@echo "" 
	arm-none-eabi-objcopy --output-target=binary $< $(TARGET).bin
	@echo "" 

flash: $(TARGET).elf
	@echo Loading binary into board
	@echo
	openocd -f ./conf/openocd.cfg -c "init;program $(TARGET).elf verify reset; exit"
	@echo

install: binary
	
	@echo Loading binary into board
	@echo
	openocd -f ./conf/openocd.cfg -c "init;program $(TARGET).bin verify reset; exit"
	@echo

clean:
	find . -name '*.o' -type f -delete
	rm $(TARGET).elf $(TARGET).map $(TARGET).bin
	@echo "" 
