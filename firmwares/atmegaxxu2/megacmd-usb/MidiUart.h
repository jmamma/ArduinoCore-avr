#pragma once

#include "megacmd-usb.h"
/**
 ** Voice category messages
 **/
#define MIDI_NOTE_OFF 0x80         /* 2 bytes data */
#define MIDI_NOTE_ON 0x90          /* 2 bytes data */
#define MIDI_AFTER_TOUCH 0xA0      /* 2 bytes data */
#define MIDI_CONTROL_CHANGE 0xB0   /* 2 bytes data */
#define MIDI_PROGRAM_CHANGE 0xC0   /* 1 byte data */
#define MIDI_CHANNEL_PRESSURE 0xD0 /* 1 byte data */
#define MIDI_PITCH_WHEEL 0xE0      /* 2 bytes data */

/**
 ** System common category messages
 **/
#define MIDI_SYSEX_START 0xF0
#define MIDI_SYSEX_END 0xF7
#define MIDI_MTC_QUARTER_FRAME 0xF1 /* 1 byte data */
#define MIDI_SONG_POSITION_PTR 0xF2 /* 2 bytes data */
#define MIDI_SONG_SELECT 0xF3       /* 1 byte data */
#define MIDI_TUNE_REQUEST 0xF6      /* no data */

/**
 ** Realtime category messages, can be sent anytime
 **/
#define MIDI_CLOCK 0xF8        /* no data */
#define MIDI_TICK 0xF9         /* no data */
#define MIDI_START 0xFA        /* no data */
#define MIDI_CONTINUE 0xFB     /* no data */
#define MIDI_STOP 0xFC         /* no data */
#define MIDI_ACTIVE_SENSE 0xFE /* no data */
#define MIDI_RESET 0xFF        /* no data */

#define MIDI_IS_STATUS_BYTE(b) ((b)&0x80)
#define MIDI_IS_VOICE_STATUS_BYTE(b) ((b) <= 0xEF)
#define MIDI_VOICE_TYPE_NIBBLE(b) ((b)&0xF0)
#define MIDI_VOICE_CHANNEL(b) ((b)&0x0F)

#define MIDI_IS_SYSCOMMON_STATUS_BYTE(b) (((b) >= 0xF0) & ((b) < 0xF8))
#define MIDI_IS_REALTIME_STATUS_BYTE(b) ((b) >= 0xF8)

class MidiUart {
public:
  uint32_t speed;

  uint16_t sendActiveSenseTimer;
  uint16_t sendActiveSenseTimeout;
  uint16_t recvActiveSenseTimer;
  bool activeSenseEnabled;

  MidiUart() {
    activeSenseEnabled = 0;
    sendActiveSenseTimeout = 0;
    recvActiveSenseTimer = 0;
    sendActiveSenseTimer = 0;
  }
  bool check_empty_tx() { return UCSR1A & (1 << UDRE1); }

  void setSpeed(uint32_t speed_) {
    Serial_Init(speed_, false);
    UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
    speed = speed_;
  }
  void m_putc(uint8_t c) {
    RingBuffer_Insert(USBtoUSART_Buffer, c);
  }

  void m_putc_immediate(uint8_t c) {
    uint8_t tmp_reg = SREG;
    cli();
    while (!check_empty_tx());
    UDR1 = c;
    sendActiveSenseTimer = sendActiveSenseTimeout;
    SREG = tmp_reg;
  }

  void setActiveSenseTimer(uint16_t timeout) {
    if (timeout == 0) {
      activeSenseEnabled = false;
    } else {
      activeSenseEnabled = true;
      sendActiveSenseTimer = 0;
      sendActiveSenseTimeout = timeout;
    }
  }

  ALWAYS_INLINE() void tickActiveSense() {
    if (recvActiveSenseTimer < 65535) {
      recvActiveSenseTimer++;
    }
    if (activeSenseEnabled) {
      if (sendActiveSenseTimer == 0) {
        m_putc(MIDI_ACTIVE_SENSE);
        sendActiveSenseTimer = sendActiveSenseTimeout;
      } else {
        sendActiveSenseTimer--;
      }
    }
  }
};

extern MidiUart uart;

