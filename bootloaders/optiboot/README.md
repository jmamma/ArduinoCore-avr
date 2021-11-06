avrdude: safemode: lfuse reads as FF
avrdude: safemode: hfuse reads as D8
avrdude: safemode: efuse reads as FF

---

Switch to 512byte boot loader

FF DE FF

Boot start address=$1FE00   BOOTSZ=11


---

DE must be set for 512 word boodloader

avrdude -c usbasp -p m2560 -U flash:w:optiboot_atmega2560.hex -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0xFF:m -U lock:w:0x0F:m -B10
