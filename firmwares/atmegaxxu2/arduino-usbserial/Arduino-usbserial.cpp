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

/** \file
 *
 *  Main source file for the Arduino-usbserial project. This file contains the
 * main tasks of the project and is responsible for the initial application
 * hardware configuration.
 */

#include "Arduino-usbserial.h"
#include "Arduino.h"
#include <util/delay.h>

/** Circular buffer to hold data from the host before it is sent to the device
 * via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the
 * host. */
RingBuff_t USARTtoUSB_Buffer;

uint32_t Boot_Key ATTR_NO_INIT;
#define MAGIC_BOOT_KEY 0xDC42ACCA
#define BOOTLOADER_START_ADDRESS 0x7000

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

void send_byte(char c) {

  if (USB_DeviceState == DEVICE_STATE_Configured) {
    RingBuffer_Insert(&USARTtoUSB_Buffer, c);
    RingBuffer_Insert(&USARTtoUSB_Buffer, '\r');
    RingBuffer_Insert(&USARTtoUSB_Buffer, '\n');
  }
}

int main(void) {
  SetupHardware();

  RingBuffer_InitBuffer(&USBtoUSART_Buffer);
  RingBuffer_InitBuffer(&USARTtoUSB_Buffer);
  sei();

  for (;;) {
    /* Let's run DFU bootloader if PC2 is active low*/
     if ((PINC & (1 << PC2)) == 0) {
        Jump_To_Bootloader();
     }

    /* Only try to read in bytes from the CDC interface if the transmit buffer
     * is not full */

    if (!(RingBuffer_IsFull(&USBtoUSART_Buffer))) {
      int16_t ReceivedByte =
          CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

      /* Read bytes from the USB OUT endpoint into the USART transmit buffer
       */
      if (!(ReceivedByte < 0))
        RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
    }
   //   if (USB_DeviceState == DEVICE_STATE_Configured) {
   //  CDC_Device_SendByte(&VirtualSerial_CDC_Interface,0xFF);

  // }

    /* Check if the UART receive buffer flush timer has expired or the buffer
     * is nearly full */
    RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
    if ((TIFR0 & (1 << TOV0)) || (BufferCount > BUFFER_NEARLY_FULL)) {
      TIFR0 |= (1 << TOV0);

      if (USARTtoUSB_Buffer.Count) {
        //   LEDs_TurnOnLEDs(LEDMASK_TX);
        //   PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
      }

      /* Read bytes from the USART receive buffer into the USB IN endpoint */
      while (BufferCount--)
        CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
                            RingBuffer_Remove(&USARTtoUSB_Buffer));

    }

    /* Load the next byte from the USART transmit buffer into the USART */
    if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer))) {
      Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));

   }

    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
  }
}
/** Configures the board hardware and chip peripherals for the demo's
 * functionality. */
void SetupHardware(void) {
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Hardware Initialization */
  Serial_Init(9600, false);
  USB_Init();

  /* Start the flush timer so that overflows occur rapidly to push received
   * bytes to the USB interface */
  TCCR0B = (1 << CS02);

  /* Set PORTC input */
  DDRC = 0;
  // PC7 is output, used for SD Card select.
  DDRC |= (1 << PC7);
  // PC2, PC4, PC5 are input and should be active low. Therefor enable pullup.
  PORTC = (1 << PC2) | (1 << PC4) | (1 << PC5);

  /* Pull target /RESET line high */
  AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
  AVR_RESET_LINE_DDR |= AVR_RESET_LINE_MASK;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
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
  if (USB_DeviceState == DEVICE_STATE_Configured)
    RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

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
