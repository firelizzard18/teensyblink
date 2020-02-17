ARDUINO=/Applications/Arduino.app/Contents/Java
ARM_BIN=$(ARDUINO)/hardware/tools/arm/bin

CC=$(ARM_BIN)/arm-none-eabi-gcc
CXX=$(ARM_BIN)/arm-none-eabi-g++
AR=$(ARM_BIN)/arm-none-eabi-gcc-ar
OBJCOPY=$(ARM_BIN)/arm-none-eabi-objcopy
OBJDUMP=$(ARM_BIN)/arm-none-eabi-objdump

FLAGS=-g -Wall -ffunction-sections -fdata-sections -nostdlib
CFLAGS=-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK66FX1M0__ -DTEENSYDUINO=146 -DARDUINO=10809 -DF_CPU=180000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH
SFLAGS=-x assembler-with-cpp $(CFLAGS)
CXXFLAGS=-fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti $(CFLAGS)
LDFLAGS=-Wl,--gc-sections,--relax -lstdc++ -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -larm_cortexM4lf_math -lm -L./build

# CORE=AudioStream.cpp DMAChannel.cpp EventResponder.cpp HardwareSerial1.cpp HardwareSerial2.cpp HardwareSerial3.cpp HardwareSerial4.cpp HardwareSerial5.cpp HardwareSerial6.cpp IPAddress.cpp IntervalTimer.cpp Print.cpp Stream.cpp Tone.cpp WMath.cpp WString.cpp analog.c avr_emulation.cpp eeprom.c keylayouts.c main.cpp math_helper.c memcpy-armv7m.S memset.S mk20dx128.c new.cpp nonstd.c pins_teensy.c ser_print.c serial1.c serial2.c serial3.c serial4.c serial5.c serial6.c serial6_lpuart.c touch.c usb_audio.cpp usb_desc.c usb_dev.c usb_flightsim.cpp usb_inst.cpp usb_joystick.c usb_keyboard.c usb_mem.c usb_midi.c usb_mouse.c usb_mtp.c usb_rawhid.c usb_seremu.c usb_serial.c usb_touch.c yield.cpp
# CORE=mk20dx128.c pins_teensy.c
CORE=mk20dx128.c

all: build/main.hex

.PHONY: all clean

clean:
	@-rm -rf build

build/%.S.o: %.S
	mkdir -p build/core
	$(CC) -c -O2 $(FLAGS) -MMD $(SFLAGS) -I./core $< -o $@

build/%.c.o: %.c core/*.h
	mkdir -p build/core
	$(CC) -c -O2 $(FLAGS) -MMD $(CFLAGS) -I./core $< -o $@

build/%.cpp.o: %.cpp core/*.h
	mkdir -p build/core
	$(CXX) -c -O2 $(FLAGS) -MMD $(CXXFLAGS) -I./core $< -o $@

build/core.a: $(patsubst %,build/core/%.o,$(CORE))
	rm $@ 2> /dev/null || true
	for x in $^; do $(AR) rcs $@ $$x; done

build/main.elf: build/main.c.o build/core.a
	$(CC) -O2 -T./core/mk66fx1m0.ld $(LDFLAGS) $^ -o $@

build/main.hex: build/main.elf
	$(OBJCOPY) -O ihex -R .eeprom $^ $@

build/main.lst: build/main.elf
	$(OBJDUMP) -d -C $^ > $@

build/main.sym: build/main.elf
	$(OBJDUMP) -t -C $^ > $@

PORT=usb:14600000
PORTLABEL=/dev/cu.usbmodem58507401 Serial
load: build/main.hex
	$(ARDUINO)/hardware/tools/teensy_post_compile -file=main -path=$(shell pwd)/build -tools=$(ARDUINO)/hardware/tools -board=TEENSY36 -reboot -port=usb:14600000 -portlabel="(null)" -portprotocol=Teensy
	# -$(ARDUINO)/hardware/tools/teensy_reboot 2> /dev/null
	# $(ARDUINO)/hardware/tools/teensy_loader_cli -mmcu=mk66fx1m0 -v -w $<