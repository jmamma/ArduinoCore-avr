#!/bin/bash

if [ -z $@ ]; then
rm out.hex

cd LUFA_100807
make clean -j 8
cd ..
cd arduino-usbdfu
make clean -j 8
make -j 8
cd ..
cd LUFA_210130
make clean -j 8
cd ..
cd megacmd-usb
make clean -j 8
make -j 8

TARGET=megacmd-usb

AVR_DIR="/Applications/Arduino.app/Contents/Java/hardware/tools/avr"
  ${AVR_DIR}/bin/avr-nm ${TARGET}.elf -Crtd --size-sort | grep -i ' [dbv] ' | sort | tail -n 40
  ram_used=$(${AVR_DIR}/bin/avr-size ${TARGET}.elf | grep ${TARGET} | awk '{ print $2 + $3}')
  ram_free=$((1024 - $ram_used))
  echo RAM_USED: $ram_used
  echo RAM_FREE: $ram_free

  cd ..
  ${AVR_DIR}/bin/avr-size arduino-usbdfu/Arduino-usbdfu.hex
  ${AVR_DIR}/bin/avr-size ${TARGET}/${TARGET}.hex
  (sed '$d' ${TARGET}/${TARGET}.hex; cat arduino-usbdfu/Arduino-usbdfu.hex) > out.hex
fi
  ${AVR_DIR}/bin/avr-size  out.hex
avrdude -c usbasp -p m32u2 -U flash:w:out.hex -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xF4:m -U lock:w:0x0F:m -B10
#avrdude -c usbasp -p m16u2 -U flash:w:out.hex -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xF4:m -U lock:w:0x0F:m -B10

