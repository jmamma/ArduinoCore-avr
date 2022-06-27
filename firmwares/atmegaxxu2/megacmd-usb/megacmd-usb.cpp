/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.

  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "Arduino.h"
#include "SDCardManager.h"
#include "megacmd-usb.h"
#include <util/delay.h>
#include "MidiUart.h"
/** Dual use buffer for MIDI and SDCard **/

#ifdef MEGACMD
uint8_t global_buffer[512];
#else
uint8_t global_buffer[sizeof(RingBuff_t) * 2];
#endif
/** Circular buffer to hold data from the host before it is sent to the device
 * via the serial port. */
RingBuff_t *USBtoUSART_Buffer = (RingBuff_t *)global_buffer;

/** Circular buffer to hold data from the serial port before it is sent to the
 * host. */
RingBuff_t *USARTtoUSB_Buffer =
    (RingBuff_t *)(global_buffer + sizeof(RingBuff_t));

MidiUart uart;

uint32_t Boot_Key ATTR_NO_INIT;
#define MAGIC_BOOT_KEY 0xDC42ACCA
#define BOOTLOADER_START_ADDRESS 0x7000

#define SET_BIT(x, y) ((x) |= (1 << (y)))
#define CLEAR_BIT(x, y) ((x) &= ~(1 << (y)))

#define UART_SET_ISR_TX_BIT() SET_BIT(UCSR1B, UDRIE1);
#define UART_CLEAR_ISR_TX_BIT() CLEAR_BIT(UCSR1B, UDRIE1);

void Bootloader_Jump_Check(void) ATTR_INIT_SECTION(3);
void Bootloader_Jump_Check(void) {
  // If the reset source was the bootloader and the key is correct, clear it and
  // jump to the bootloader
  if ((MCUSR & (1 << WDRF)) && (Boot_Key == MAGIC_BOOT_KEY)) {
    Boot_Key = 0;
    ((void (*)(void))BOOTLOADER_START_ADDRESS)();
  }
}

void Jump_To_Bootloader(void) {
  // If USB is used, detach from the bus and reset it
  USB_Disable();

  // Disable all interrupts
  cli();

  // Wait two seconds for the USB detachment to register on the host
  for (uint8_t i = 0; i < 128; i++)
    _delay_ms(16);

  // Set the bootloader key to the magic value and force a reset
  Boot_Key = MAGIC_BOOT_KEY;
  wdt_enable(WDTO_250MS);
  for (;;)
    ;
}
extern "C" void uart_tick();

void uart_tick() { uart.tickActiveSense(); }

/** Configures the board hardware and chip peripherals for the demo's
 * functionality. */
void SetupHardware(uint8_t state) {
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  switch (state) {
  case USB_SERIAL:
    uart.setSpeed(9600);
    break;
  case USB_MIDI:
    uart.setSpeed(31250);
    break;
  case USB_STORAGE:
#ifdef MEGACMD
    while (!SDCardManager_Init(8))
      ;
#endif
    break;
  }

  usb_mode = state;

  USB_Init();

  /* Start the flush timer so that overflows occur rapidly to push received
   * bytes to the USB interface */
  TCCR0B = (1 << CS02);

  /* Pull target /RESET line high */
  if (usb_mode == USB_SERIAL) {
    AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
    AVR_RESET_LINE_DDR |= AVR_RESET_LINE_MASK;
  }
}

void initState(uint8_t state) {
  SetupHardware(state);

  RingBuffer_InitBuffer(USBtoUSART_Buffer);
  RingBuffer_InitBuffer(USARTtoUSB_Buffer);
  sei();
  // wait for MegaCMD to boot
  for (uint8_t i = 0; i < 128; i++)
    _delay_ms(16);
}

void send_midi_event(MIDI_EventPacket_t *event) {
  MIDI_Device_SendEventPacket(&USB_MIDI_Interface, event);
  MIDI_Device_Flush(&USB_MIDI_Interface);
}

void switchState(uint8_t state) {
  if (state > 3) {
    return;
  }

  if (state == USB_DFU) {
    Jump_To_Bootloader();
  }
  Serial_Disable();
  USB_Detach();
  cli();
  USB_Disable();
  // Serial_Disable();
  for (uint8_t i = 0; i < 8; i++)
    _delay_ms(16);
  initState(state);
}

int main(void) {

  uint8_t state = USB_MIDI;

  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  // 1000 Hz (16000000/((249+1)*64))
  OCR1A = 249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  /* Set PORTC input */

  // PC2 -> PK0
  // PC4 -> PK1
  // PC5 -> PK2
  DDRC = 0;
  PORTC = 0;
  // PC7 is output, used for SD Card select. Active HIGH
  // PC5 is output, used for indicating official MegaCMD. active low
  DDRC |= (1 << PC7) | (1 << PC5);
  PORTC |= (1 << PC7);
  // PC4, PC2 are input. DFU mode is active low. Therefor enable pullup.
  PORTC |= (1 << PC2) | (1 << PC4);

INIT:
  initState(state);

  for (;;) {
#ifdef MEGACMD
    bool a = PINC & (1 << PC5);
    bool b = PINC & (1 << PC4);

    state = (uint8_t)a * 2 + (uint8_t)b;
    if (state == USB_DFU) {
      switchState(state);
    }
#endif
    if (USB_DeviceState == DEVICE_STATE_Configured) {
      switch (usb_mode) {
      case USB_SERIAL:
        USB_Serial();
        break;
      case USB_MIDI:
        USB_Midi();
        break;
      case USB_STORAGE:
#ifdef MEGACMD
        MS_Device_USBTask(&Disk_MS_Interface);
#endif
        break;
      }
      USB_USBTask();
    }
  }
}

void USB_Serial() {

  /* Only try to read in bytes from the CDC interface if the transmit buffer
   * is not full */

  if (!(RingBuffer_IsFull(USBtoUSART_Buffer))) {
    int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);


    if (!(ReceivedByte < 0)) {
      RingBuffer_Insert(USBtoUSART_Buffer, ReceivedByte);
      UART_SET_ISR_TX_BIT();
    }
  }
  RingBuff_Count_t BufferCount = RingBuffer_GetCount(USARTtoUSB_Buffer);
  if (BufferCount) {
    Endpoint_SelectEndpoint(
        VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

    if (Endpoint_IsINReady()) {
      uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

      while (BytesToSend--) {
        if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
                                RingBuffer_Peek(USARTtoUSB_Buffer)) !=
            ENDPOINT_READYWAIT_NoError) {
          break;
        }

        RingBuffer_Remove(USARTtoUSB_Buffer);
      }
    }
  }

  /* Load the next byte from the USART transmit buffer into the USART */

  CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
}

int8_t message_len = 0;
uint8_t data_cnt = 0;
bool in_sysex = 0;
bool special_case = false;
MIDI_EventPacket_t SendMIDIEvent;

const uint8_t change_mode_msg[] PROGMEM = {0xF0, 0x7D, 0x4D, 0x43,
                                           0x4C, 0x01, 0xFF, 0xF7};

const uint8_t turbo_setspeed_msg[] PROGMEM = {0xF0, 0x00, 0x20, 0x3c, 0x00,
                                              0x00, 0x20, 0xFF, 0xF7};

class MessageCheck {

public:
  MessageCheck(uint8_t *msg_, uint8_t len_, void (*callback_)(uint8_t)) {
    msg = msg_;
    len = len_;
    callback = callback_;
  }
  uint8_t *msg;
  uint8_t count = 0;
  uint8_t state = 0;
  uint8_t len;
  void (*callback)(uint8_t);

  bool check(uint8_t byte) {
    uint8_t b = pgm_read_byte_near(msg + count);
    if (b == 0xFF || byte == b) {
      if (b == 0xFF) {
        state = byte;
      }
      count++;
      if (count == len) {
        callback(state);
        count = 0;
        return true;
      }
    } else {
      count = 0;
    }
    return false;
  }
};

MessageCheck msg_usb(change_mode_msg, sizeof(change_mode_msg), &switchState);
MessageCheck msg_uart(change_mode_msg, sizeof(change_mode_msg), &switchState);

void turbo_set_speed(uint8_t speed) {
  const uint32_t tmSpeeds[] = {31250,  31250,  62500,  104062, 125000, 156250,
                               208125, 250000, 312500, 415625, 500000, 625000};

  if (uart.speed == tmSpeeds[speed]) {
    return;
  }
  cli();
  for (uint8_t n = 0; n < 16; n++) {
    uart.m_putc_immediate(0x00);
  }
  uart.setSpeed(tmSpeeds[speed]);
  // RingBuffer_InitBuffer(USARTtoUSB_Buffer);
  // RingBuffer_InitBuffer(USBtoUSART_Buffer);
  if (speed <= 1) {
    uart.activeSenseEnabled = false;
  } else {
    uart.setActiveSenseTimer(150);
  }
  sei();
}

MessageCheck msg_uart_turbo(turbo_setspeed_msg, sizeof(turbo_setspeed_msg),
                            &turbo_set_speed);

void USB_Midi() {

  /*if (uart.activeSenseEnabled && uart.recvActiveSenseTimer >= 300) {
    turbo_set_speed(0);
  }*/

  MIDI_EventPacket_t ReceivedMIDIEvent;

  while (
      MIDI_Device_ReceiveEventPacket(&USB_MIDI_Interface, &ReceivedMIDIEvent)) {
    uint8_t *msg = ((uint8_t *)&ReceivedMIDIEvent);

    uint8_t cin = (*msg) & 0x0F;
    uint8_t len = 0;
    msg++;
    switch (cin) {
    default: {
      continue;
    }
    case 0x5:
    case 0xF: {
      len = 1;
      break;
    }
    case 0x2:
    case 0x6:
    case 0xC:
    case 0xD: {
      len = 2;
      break;
    }
    case 0x3:
    case 0x4:
    case 0x7:
    case 0x8:
    case 0x9:
    case 0xA:
    case 0xB:
    case 0xE: {
      len = 3;
      break;
    }
    }

    for (uint8_t n = 0; n < len; n++) {
      msg_usb.check(msg[n]);
      RingBuffer_Insert(USBtoUSART_Buffer, msg[n]);
    }
    if (len) {
      UART_SET_ISR_TX_BIT();
    }
  }
  uint16_t BufferCount = RingBuffer_GetCount(USARTtoUSB_Buffer);
  Endpoint_SelectEndpoint(USB_MIDI_Interface.Config.DataINEndpoint.Address);

  if (BufferCount && Endpoint_IsINReady()) {
    uint8_t count = MIN(BufferCount, (CDC_TXRX_EPSIZE - 4));
    uint8_t *ptr = ((uint8_t *)&SendMIDIEvent);

    while (count--) {
      uint8_t c = RingBuffer_Remove(USARTtoUSB_Buffer);
      bool send = false;

      // REALTIME, >= 0xF8
      if (MIDI_IS_REALTIME_STATUS_BYTE(c)) {
        uart.recvActiveSenseTimer = 0;
        switch (c) {
        default:
          MIDI_EventPacket_t SendMIDIEventTemp;
          ptr = ((uint8_t *)&SendMIDIEventTemp);
          ptr[0] = 0xF; // Single byte, Realtime
          ptr[1] = c;
          ptr[2] = 0;
          ptr[3] = 0;
          send_midi_event(&SendMIDIEventTemp);
          continue;
        case MIDI_ACTIVE_SENSE:
          // Ignore active sense
          continue;
        }
      }
      // STATUS BYTES < 0xF0
      if (MIDI_IS_STATUS_BYTE(c)) {
        message_len = -1;
        uart.recvActiveSenseTimer = 0;
        if (c < 0xF0) {
          in_sysex = 0;
          switch (c & 0xF0) {
          case MIDI_NOTE_OFF:
            ptr[0] = 0x8;
            message_len = 2;
            break;
          case MIDI_NOTE_ON:
            ptr[0] = 0x9;
            message_len = 2;
            break;
          case MIDI_AFTER_TOUCH:
            ptr[0] = 0xA;
            message_len = 2;
            break;
          case MIDI_CONTROL_CHANGE:
            ptr[0] = 0xB;
            message_len = 2;
            break;
          case MIDI_CHANNEL_PRESSURE:
            ptr[0] = 0xC;
            message_len = 1;
            break;
          case MIDI_PROGRAM_CHANGE:
            ptr[0] = 0xD;
            message_len = 1;
            break;
          case MIDI_PITCH_WHEEL:
            ptr[0] = 0xE;
            message_len = 2;
            break;
          }
        } else {
          // System common 0xF0 -> 0xF7
          switch (c) {
          default:
            ptr[0] = 0x5;
            in_sysex = 0;
            special_case = false;
            data_cnt = 0;
            message_len = 0;
            break;
          case MIDI_MTC_QUARTER_FRAME:
            ptr[0] = 0x2;
            message_len = 1;
            break;
          case MIDI_SONG_POSITION_PTR:
            ptr[0] = 0x3;
            message_len = 2;
            break;
          case MIDI_SONG_SELECT:
            ptr[0] = 0x2;
            message_len = 1;
            break;
          case MIDI_SYSEX_START:
            ptr[0] = 0x4;
            in_sysex = 1;
            special_case = true;
            uart.recvActiveSenseTimer = 0;
            break;
          case MIDI_SYSEX_END:
            uart.recvActiveSenseTimer = 0;
            if (!in_sysex)
              break; // error
            if (msg_uart.check(c)) {
              return;
            }
            msg_uart_turbo.check(c);
            in_sysex = 0;
            send = true;
            if (special_case) {
              if (data_cnt == 1) {
                ptr[0] = 0x6;
                ptr[1] = 0xF0;
                ptr[2] = 0xF7;
                ptr[3] = 0x00;
              }
              if (data_cnt == 2) {
                ptr[0] = 0x7;
                ptr[1] = 0xF0;
                ptr[3] = 0xF7;
              }
            } else {
              if (data_cnt == 0) {
                ptr[0] = 0x5;
                ptr[1] = 0xF7;
                ptr[2] = 0x00;
                ptr[3] = 0x00;
              }
              if (data_cnt == 1) {
                ptr[0] = 0x6;
                ptr[2] = 0xF7;
                ptr[3] = 0x00;
              }
              if (data_cnt == 2) {
                ptr[0] = 0x7;
                ptr[3] = 0xF7;
              }
            }
            break;
          }
        }
        data_cnt = 0;

        if (message_len >= 0) {
          in_sysex = 0;
          ptr[1] = c;
          ptr[2] = 0;
          ptr[3] = 0;
          data_cnt++;
        }
        if (message_len == 0) {
          send = true; // 0 data message. send immediately.
        }
      } else if (message_len > 0) {
        ptr[1 + data_cnt] = c;
        data_cnt++;
        message_len--;
        if (message_len == 0) {
          send = true;
        }
      }
      // Data
      if (in_sysex) {
        uart.recvActiveSenseTimer = 0;
        msg_uart.check(c);
        msg_uart_turbo.check(c);
        ptr[1 + data_cnt] = c;
        data_cnt++;
        if (data_cnt == 3) {
          data_cnt = 0;
          ptr[0] = 0x4; // Sysex continues;
          send = true;
          special_case = false;
        }
      }
      if (send) {
        send_midi_event(&SendMIDIEvent);
      }
    }
  }
  /* General management task for a given MIDI class interface, required for
   * the correct operation of the interface.
   * This should be called frequently in the main program loop, before the
   * master USB management task USB_USBTask(). */
  MIDI_Device_USBTask(&USB_MIDI_Interface);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {

  switch (usb_mode) {
  case USB_SERIAL:
    CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
    break;
  case USB_MIDI:
    MIDI_Device_ConfigureEndpoints(&USB_MIDI_Interface);
    break;
  case USB_STORAGE:
#ifdef MEGACMD
    MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
#endif
    break;
  }
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
  switch (usb_mode) {
  case USB_SERIAL:
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
    break;
  case USB_MIDI:
    MIDI_Device_ProcessControlRequest(&USB_MIDI_Interface);
    break;
  case USB_STORAGE:
#ifdef MEGACMD
    MS_Device_ProcessControlRequest(&Disk_MS_Interface);
#endif
    break;
  }
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface
 * configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(
    USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo) {
  uint8_t ConfigMask = 0;

  switch (CDCInterfaceInfo->State.LineEncoding.ParityType) {
  case CDC_PARITY_Odd:
    ConfigMask = ((1 << UPM11) | (1 << UPM10));
    break;
  case CDC_PARITY_Even:
    ConfigMask = (1 << UPM11);
    break;
  }

  if (CDCInterfaceInfo->State.LineEncoding.CharFormat ==
      CDC_LINEENCODING_TwoStopBits)
    ConfigMask |= (1 << USBS1);

  switch (CDCInterfaceInfo->State.LineEncoding.DataBits) {
  case 6:
    ConfigMask |= (1 << UCSZ10);
    break;
  case 7:
    ConfigMask |= (1 << UCSZ11);
    break;
  case 8:
    ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
    break;
  }

  /* Must turn off USART before reconfiguring it, otherwise incorrect
   * operation may occur */
  UCSR1B = 0;
  UCSR1A = 0;
  UCSR1C = 0;

  /* Special case 57600 baud for compatibility with the ATmega328 bootloader.
   */
  UBRR1 =
      (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
          ? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
          : SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

  UCSR1C = ConfigMask;
  UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
               ? 0
               : (1 << U2X1);
  UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}

/** ISR to manage the reception of data from the serial port, placing received
 * bytes into a circular buffer for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK) {
  uint8_t ReceivedByte = UDR1;
  if (USB_DeviceState == DEVICE_STATE_Configured && !(RingBuffer_IsFull(USARTtoUSB_Buffer))) {
    RingBuffer_Insert(USARTtoUSB_Buffer, ReceivedByte);
  }
}

ISR(USART1_UDRE_vect, ISR_BLOCK) {
  if (!(RingBuffer_IsEmpty(USBtoUSART_Buffer))) {
    UDR1 = RingBuffer_Remove(USBtoUSART_Buffer);
  }
  if ((RingBuffer_IsEmpty(USBtoUSART_Buffer))) {
    UART_CLEAR_ISR_TX_BIT();
  }
}
void EVENT_USB_Device_Disconnect(void) {}
/** Event handler for the CDC Class driver Host-to-Device Line Encoding
 * Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface
 * configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(
    USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo) {
  bool CurrentDTRState =
      (CDCInterfaceInfo->State.ControlLineStates.HostToDevice &
       CDC_CONTROL_LINE_OUT_DTR);

  if (CurrentDTRState)
    AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
  else
    AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
}

bool CALLBACK_MS_Device_SCSICommandReceived(
    USB_ClassInfo_MS_Device_t *const MSInterfaceInfo) {
  bool CommandSuccess;

  CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);

  return CommandSuccess;
}
