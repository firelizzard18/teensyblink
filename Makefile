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
LDFLAGS=-Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1579708172 -lstdc++ -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -larm_cortexM4lf_math -lm -L./build

all: build/main.hex

.PHONY: all clean

clean:
	@-rm -rf build

build/%.S.o: %.S
	mkdir -p build/core
	$(CC) -c -O2 $(FLAGS) -MMD $(SFLAGS) -I./core $< -o $@

build/%.c.o: %.c
	mkdir -p build/core
	$(CC) -c -O2 $(FLAGS) -MMD $(CFLAGS) -I./core $< -o $@

build/%.cpp.o: %.cpp
	mkdir -p build/core
	$(CXX) -c -O2 $(FLAGS) -MMD $(CXXFLAGS) -I./core $< -o $@

build/core.a: build/core/AudioStream.cpp.o build/core/DMAChannel.cpp.o build/core/EventResponder.cpp.o build/core/HardwareSerial1.cpp.o build/core/HardwareSerial2.cpp.o build/core/HardwareSerial3.cpp.o build/core/HardwareSerial4.cpp.o build/core/HardwareSerial5.cpp.o build/core/HardwareSerial6.cpp.o build/core/IPAddress.cpp.o build/core/IntervalTimer.cpp.o build/core/Print.cpp.o build/core/Stream.cpp.o build/core/Tone.cpp.o build/core/WMath.cpp.o build/core/WString.cpp.o build/core/analog.c.o build/core/avr_emulation.cpp.o build/core/eeprom.c.o build/core/keylayouts.c.o build/core/main.cpp.o build/core/math_helper.c.o build/core/memcpy-armv7m.S.o build/core/memset.S.o build/core/mk20dx128.c.o build/core/new.cpp.o build/core/nonstd.c.o build/core/pins_teensy.c.o build/core/ser_print.c.o build/core/serial1.c.o build/core/serial2.c.o build/core/serial3.c.o build/core/serial4.c.o build/core/serial5.c.o build/core/serial6.c.o build/core/serial6_lpuart.c.o build/core/touch.c.o build/core/usb_audio.cpp.o build/core/usb_desc.c.o build/core/usb_dev.c.o build/core/usb_flightsim.cpp.o build/core/usb_inst.cpp.o build/core/usb_joystick.c.o build/core/usb_keyboard.c.o build/core/usb_mem.c.o build/core/usb_midi.c.o build/core/usb_mouse.c.o build/core/usb_mtp.c.o build/core/usb_rawhid.c.o build/core/usb_seremu.c.o build/core/usb_serial.c.o build/core/usb_touch.c.o build/core/yield.cpp.o
	rm $@ 2> /dev/null; for x in $^; do $(AR) rcs $@ $$x; done

build/main.elf: build/main.cpp.o build/core.a
	$(CC) -O2 -T./core/mk66fx1m0.ld $(LDFLAGS) $^ -o $@

# build/main.eep: build/main.elf
# 	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $^ $@

build/main.hex: build/main.elf
	$(OBJCOPY) -O ihex -R .eeprom $^ $@

build/main.lst: build/main.elf
	$(OBJDUMP) -d -S -C $^ > $@

build/main.sym: build/main.elf
	$(OBJDUMP) -t -C $^ > $@

PORT=usb:14600000
PORTLABEL=/dev/cu.usbmodem58507401 Serial
load: build/main.hex
	$(ARDUINO)/hardware/tools/teensy_post_compile -file=main -path=$(shell pwd)/build -tools=$(ARDUINO)/hardware/tools -board=TEENSY36 -reboot -port=$(PORT) -portlabel=$(PORTLABEL) -portprotocol=Teensy
	# $(ARDUINO)/hardware/tools/teensy_loader_cli -mmcu=mk66fx1m0 -v -w $<