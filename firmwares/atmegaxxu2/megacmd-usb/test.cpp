#include "inttypes.h"
#include "stdio.h"

/**
 ** Voice category messages
 **/
#define MIDI_NOTE_OFF 0x80         /* 2 uint8_ts data */
#define MIDI_NOTE_ON 0x90          /* 2 uint8_ts data */
#define MIDI_AFTER_TOUCH 0xA0      /* 2 uint8_ts data */
#define MIDI_CONTROL_CHANGE 0xB0   /* 2 uint8_ts data */
#define MIDI_PROGRAM_CHANGE 0xC0   /* 1 uint8_t data */
#define MIDI_CHANNEL_PRESSURE 0xD0 /* 1 uint8_t data */
#define MIDI_PITCH_WHEEL 0xE0      /* 2 uint8_ts data */

/**
 ** System common category messages
 **/
#define MIDI_SYSEX_START 0xF0
#define MIDI_SYSEX_END 0xF7
#define MIDI_MTC_QUARTER_FRAME 0xF1 /* 1 uint8_t data */
#define MIDI_SONG_POSITION_PTR 0xF2 /* 2 uint8_ts data */
#define MIDI_SONG_SELECT 0xF3       /* 1 uint8_t data */
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

int8_t message_len = 0;
uint8_t data_cnt = 0;
bool in_sysex = 0;
bool special_case = false;
uint8_t out[4];

int main() {
  uint8_t *ptr = out;
  //uint8_t payload[] = {0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0xF7};
  //uint8_t payload[] = { 0xB0, 0x01, 0x02 };

  //uint8_t payload[] = {0xF0, 0x01, 0x02, 0x03, 0xF7};
  uint8_t payload[] = { 0xF8 };
  for (uint8_t n = 0; n < sizeof(payload); n++) {
    uint8_t c = payload[n];
    printf("%02X\n", c);
    bool send = false;
    // STATUS BYTES
    if (MIDI_IS_STATUS_BYTE(c)) {
      message_len = -1;
      if (c < 0xF0) {
        switch (c & 0xF0) {
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
        case MIDI_TUNE_REQUEST:
          ptr[0] = 0x5;
          message_len = 0;
          break;
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
        switch (c) {
        default:
          ptr[0] = 0xF; // Single uint8_t, Realtime
          message_len = 0;
          break;
        case MIDI_SYSEX_START:
          ptr[0] = 0x4;
          in_sysex = 1;
          special_case = true;
          break;
        case MIDI_SYSEX_END:
          if (!in_sysex)
            break; // error
          printf("end %d\n", data_cnt);
          in_sysex = 0;
          send = true;
          if (special_case) {
            printf("special");
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
      printf("m ptr[%d] = %02X\n", 1 + data_cnt, c);
      data_cnt++;
      message_len--;
      printf("message_len %d\n", message_len);
      if (message_len == 0) {
        send = true;
      }
    }
    // Data
    if (in_sysex) {
      ptr[1 + data_cnt] = c;
      printf("s ptr[%d] = %02X\n", 1 + data_cnt, c);
      data_cnt++;
      if (data_cnt == 3) {
        data_cnt = 0;
        ptr[0] = 0x4; // Sysex continues;
        send = true;
        special_case = false;
      }
    }
    if (send) {
      printf("packet: %02X %02X %02X %02X\n", ptr[0], ptr[1], ptr[2], ptr[3]);
    }
  }
}
