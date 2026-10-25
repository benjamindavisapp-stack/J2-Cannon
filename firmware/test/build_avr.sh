#!/usr/bin/env bash
# Compile the sketch for the RedBoard (ATmega328P) without the Arduino IDE,
# to check that it builds and fits. Uses avr-gcc and an Arduino AVR core
# (set ARDUINO_AVR if yours is not in the Debian location).
set -euo pipefail
cd "$(dirname "$0")"
CORE=${ARDUINO_AVR:-/usr/share/arduino/hardware/arduino/avr}
OUT=build-avr
rm -rf "$OUT" && mkdir -p "$OUT"

COMMON="-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -Os -g -ffunction-sections -fdata-sections"
INC="-I$CORE/cores/arduino -I$CORE/variants/standard -I$CORE/libraries/Wire/src -I$CORE/libraries/Wire/src/utility -I$CORE/libraries/EEPROM/src -I../j2_cannon"
CXX="avr-g++ $COMMON $INC -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics"
CC="avr-gcc $COMMON $INC -std=gnu11"

for f in "$CORE"/cores/arduino/*.c "$CORE"/libraries/Wire/src/utility/twi.c; do $CC -w -c "$f" -o "$OUT/core_$(basename "$f").o"; done
for f in "$CORE"/cores/arduino/*.cpp "$CORE"/libraries/Wire/src/Wire.cpp; do $CXX -w -c "$f" -o "$OUT/core_$(basename "$f").o"; done
for f in "$CORE"/cores/arduino/*.S; do [ -e "$f" ] && avr-gcc $COMMON -x assembler-with-cpp -c "$f" -o "$OUT/core_$(basename "$f").o"; done

for f in ../j2_cannon/*.cpp; do $CXX -Wall -Wextra -c "$f" -o "$OUT/$(basename "$f").o"; done
$CXX -Wall -Wextra -x c++ -c ../j2_cannon/j2_cannon.ino -o "$OUT/j2_cannon.ino.o"

avr-gcc -mmcu=atmega328p -Os -Wl,--gc-sections -o "$OUT/j2_cannon.elf" "$OUT"/*.o -lm
avr-objcopy -O ihex -R .eeprom "$OUT/j2_cannon.elf" "$OUT/j2_cannon.hex"
avr-size -C --mcu=atmega328p "$OUT/j2_cannon.elf"
