PDEDIR ?= .

ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

ARDUINO_MODEL_PID = 0x0001
BOARD = USER

TARGET = Arduino-usbserial

FORMAT = ihex

BASE_DIR= /Applications/Arduino.app/Contents/Java/hardware
CORE_DIR=$(shell pwd)
VARIENTS_DIR=$(shell cd ../../; pwd)/variants/mega

AR=$(shell which avr-gcc-ar)
CC=$(shell which avr-gcc)
CXX=$(shell which avr-g++)
OBJCOPY=$(shell which avr-objcopy)
AVRSIZE=$(shell which avr-size)
AVRDUDE=$(shell which avrdude)
AVRDUDE_CFG=/etc/avrdude.conf

#fallback if system avr - gcc is not detected
ifeq (${CC},)
AR=${BASE_DIR}/tools/avr/bin/avr-gcc-ar
CC=${BASE_DIR}/tools/avr/bin/avr-gcc
CXX=${BASE_DIR}/tools/avr/bin/avr-g++
OBJCOPY =${BASE_DIR}/tools/avr/bin/avr-objcopy
AVRSIZE=${BASE_DIR}/tools/avr/bin/avr-size
AVRDUDE=${BASE_DIR}/tools/avr/bin/avrdude
AVRDUDE_CFG=${BASE_DIR}/tools/avr/etc/avrdude.conf
endif

AVR_DEVICE=$(shell ls /dev/tty* | grep -i usb | head -n 1)
UISP=uisp
AVR_ARCH = atmega32u2
F_CPU = 16000000L
F_CLOCK = $(F_CPU)

INOFILES=$(wildcard *.pde)

OPTIM_FLAGS   = -Os
WARN_FLAGS    = -Wall -Wextra
COMPILE_FLAGS = -c $(OPTIM_FLAGS) $(WARN_FLAGS) -MMD
FEATURE_FLAGS = -ffunction-sections -fdata-sections -flto=jobserver
LINK_FLAGS    = -lm -Wl,--gc-sections,--relax
ARCH_FLAGS    = -mmcu=atmega32u2 -DF_CPU=16000000L -DARDUINO=10803 -DARDUINO_AVR_ATmega32U2 -DARDUINO_ARCH_AVR $(LUFA_OPTS)
ARCH_FLAGS += -DARDUINO_MODEL_PID=$(ARDUINO_MODEL_PID)
ARCH_FLAGS += -DBOARD=BOARD_$(BOARD)
ARCH_FLAGS += -DF_CLOCK=$(F_CLOCK)
ARCH_FLAGS += -DAVR_RESET_LINE_PORT="PORTD"
ARCH_FLAGS += -DAVR_RESET_LINE_DDR="DDRD"
ARCH_FLAGS += -DAVR_RESET_LINE_MASK="(1 << 7)"
ARCH_FLAGS += -DTX_RX_LED_PULSE_MS=3
ARCH_FLAGS += -DPING_PONG_LED_PULSE_MS=100
ARCH_FLAGS += -DARDUINO=1

CFLAGS	  = -std=gnu11 $(COMPILE_FLAGS) $(FEATURE_FLAGS) $(LINK_FLAGS) $(ARCH_FLAGS)
CXXFLAGS	= -std=gnu++1z $(COMPILE_FLAGS) $(FEATURE_FLAGS) $(LINK_FLAGS) $(ARCH_FLAGS)
CXXFLAGS     += -fpermissive -fno-exceptions -fno-threadsafe-statics -fno-rtti

LDFLAGS = $(OPTIM_FLAGS) $(WARN_FLAGS) -flto -fuse-linker-plugin -mrelax $(LINK_FLAGS) $(ARCH_FLAGS)

DIRS = SdFat SdFat/SdCard SdFat/FatLib SdFat/SpiDriver SdFat/SpiDriver/boards MCL SPI $(LUFA_PATH)

LDIRS = $(foreach dir,$(DIRS),$(CORE_DIR)/$(dir)) ${CORE_DIR} ${VARIENTS_DIR}

$(info  LDIRS is $(LDIRS))
INCS = -I. $(foreach dir,$(LDIRS),-I$(dir))
OBJS = $(foreach dir,$(LDIRS),$(foreach file,$(wildcard $(dir)/*.cpp),$(subst .cpp,.o,$(file))))
OBJS += $(foreach dir,$(LDIRS),$(foreach file,$(wildcard $(dir)/*.c),$(subst .c,.o,$(file))))


# Path to the LUFA library
LUFA_PATH = ../LUFA_100807


# LUFA library compile-time options
LUFA_OPTS  = -D USB_DEVICE_ONLY
LUFA_OPTS += -D FIXED_CONTROL_ENDPOINT_SIZE=8
LUFA_OPTS += -D FIXED_NUM_CONFIGURATIONS=1
LUFA_OPTS += -D USE_FLASH_DESCRIPTORS
LUFA_OPTS += -D INTERRUPT_CONTROL_ENDPOINT
LUFA_OPTS += -D DEVICE_STATE_AS_GPIOR=0
LUFA_OPTS += -D USE_STATIC_OPTIONS="(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)"

# Create the LUFA source path variables by including the LUFA root makefile
include $(LUFA_PATH)/LUFA/makefile

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c\
	Descriptors.c\
    $(LUFA_SRC_USB) \
	$(LUFA_SRC_USBCLASS)    \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/Device.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/Endpoint.c	     \
	$(LUFA_PATH)/LUFA/Drivers/USB/HighLevel/HostStandardReq.c   \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/Host.c   \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/Pipe.c   \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/USBController.c	\
	$(LUFA_PATH)/LUFA/Drivers/USB/HighLevel/Events.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/LowLevel/USBInterrupt.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/HighLevel/USBTask.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/HighLevel/DeviceStandardReq.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/HighLevel/ConfigDescriptor.c  \
	$(LUFA_PATH)/LUFA/Drivers/USB/Class/Device/CDC.c \
	$(LUFA_PATH)/LUFA/Drivers/USB/Class/Host/CDC.c 

OBJS += $(SRC:%.c=./%.o) $(CPPSRC:%.cpp=./%.o)  

#$(info    OBJS is $(OBJS))
DFILES = $(foreach obj,$(OBJS),$(subst .o,.d,$(obj)))
CFLAGS += $(INCS)
CXXFLAGS += $(INCS)

CFLAGS += -DMCL_NOSKETCH
CXXFLAGS += -DMCL_NOSKETCH

all: $(TARGET).hex

%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock $< $@

$(TARGET).elf: core.a
	$(CC) $(LDFLAGS) -o $@ $^

core.a: $(OBJS) Makefile
	echo $(AR) $@ $^
	$(AR) rcs $@ $^

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp Makefile
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.s Makefile
	$(CC) $(CFLAGS) -c $< -o $@

%.s: %.c
	$(CC) -S $(CFLAGS) -fverbose-asm $< -o $@

%.s: %.cpp
	$(CXX) -S $(CXXFLAGS) -fverbose-asm $< -o $@

%.os: %.o
	avr-objdump -S $< > $@

%.elfs: %.elf
	avr-objdump -S $< > $@

%.o: %.S
	$(CC) $(CFLAGS) -Wa,-adhlns=$@.lst -c $< -o $@

%.d:%.c
	set -e; $(CC) -MM $(CFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@ ; \
	[ -s $@ ] || rm -f $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@; \
	$(AVRSIZE) $<

%.ee_srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@


libclean:
	rm -rf $(OBJS)
	rm -rf $(DFILES)

clean:
	find . -iname '*.d' -exec rm {} \; ; \
	find . -iname '*.o' -exec rm {} \; ; \
	find . -iname '*.os' -exec rm {} \; ; \
	find . -iname '*.elf' -exec rm {} \; ; \
	find . -iname '*.lst' -exec rm {} \; ;


upload: main.hex
	echo "DEVICE is $(AVR_DEVICE)"; \
  echo "RAM USAGE: `$(AVRSIZE) main.elf | grep main | awk '{ print $$2 + $$3}'`"; \
  ${AVRDUDE} -C${AVRDUDE_CFG} -v -V -patmega2560 -cwiring -P${AVR_DEVICE} -b115200 -D -Uflash:w:./main.hex

debug: CXXFLAGS += -DDEBUGMODE=1
debug: CFLAGS += -DDEBUGMODE=1
debug: main.hex

debug_upload: debug upload
