AS = arm-none-eabi-as
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
BIN = arm-none-eabi-objcopy
CFLAGS = -mthumb -mcpu=cortex-m4

all: main.bin

crt.o: crt.s
	$(AS) -o crt.o crt.s

main.o: main.c
	$(CC) $(CFLAGS) -c -o main.o main.c

main.elf: linker.ld crt.o main.o
	$(LD) -T linker.ld -o main.elf crt.o main.o

main.bin: main.elf
	$(BIN) -O binary main.elf main.bin

flash-ftdi: all
	stm32flash -g 0x0 -w main.bin /dev/ttyUSB0

flash-usb: all
	dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D main.bin

list:
	dfu-util -l

clean:
	rm crt.o main.o main.elf main.bin