#!/bin/bash
rm out.hex
cd arduino-usbdfu
make
cd ..
cd arduino-usbserial
make
AVR_DIR="/Applications/Arduino.app/Contents/Java/hardware/tools/avr"
  ${AVR_DIR}/bin/avr-nm Arduino-usbserial.elf -Crtd --size-sort | grep -i ' [dbv] ' | sort | tail -n 40
  ram_used=$(${AVR_DIR}/bin/avr-size Arduino-usbserial.elf | grep Arduino-usbserial | awk '{ print $2 + $3}')
  ram_free=$((1024 - $ram_used))
  echo RAM_USED: $ram_used
  echo RAM_FREE: $ram_free
cd ..
sed '$d' arduino-usbdfu/Arduino-usbdfu.hex > out.hex
cat arduino-usbserial/Arduino-usbserial.hex >> out.hex
avrdude -c usbasp -p m32u2 -U flash:w:out.hex -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xF4:m -U lock:w:0x0F:m -B10

