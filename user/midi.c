/* ********************************************************************
 *
 *  Copyright (c) 2019 Christian Haettich [feddischson@gmail.com]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#include "midi.h"
#include "user_config.h"

/* Internal data and functions */
#define MIDI_ST_INIT 0
#define MIDI_ST_IDLE 1
#define MIDI_ST_WAIT_D1 2
#define MIDI_ST_WAIT_D2 3
#define MIDI_ST_WAIT_F7 4

/** @brief Size of the midi message buffer
 *  @details
 *    The buffer is larger than 3 bytes in order to handle
 *    0xf0 xx xx xx xx xx xx 0xf7 messages, where there
 *    is an unlimited number of xx bytes
 *    (system exclusive messages).
 */
#define MIDI_MSG_BUF_SIZE 4096

struct _midi_data {
   uint8_t initialized;   /**< Init-flag: 1=struct is initialized */
   uint8_t state;         /**< Current processing state */
   uint8_t expected_data; /**< Length of the expected data */
   uint8_t message[MIDI_MSG_BUF_SIZE]; /**< Message buffer */
   uint16_t byte_cnt; /**< Counter for f0/f7 encapsulated messages */
   midi_done_cb_t cb; /**< Call-back function */
} midi_data = {0, MIDI_ST_INIT, 0, {0}, 0, 0};

/** @brief Sets the first byte and updates the state
 *  @param status_byte The status byte.
 *  @param pytes_pending True if more bytes are in the uart fifo.
 */
static void ICACHE_FLASH_ATTR midi_set_status(uint8_t status_byte,
                                              uint8_t bytes_pending);

/** @brief Sets the first data byte and updates the state
 *  @param status_byte The status byte.
 *  @param pytes_pending True if more bytes are in the uart fifo.
 */
static void ICACHE_FLASH_ATTR midi_set_d1(uint8_t data_byte,
                                          uint8_t bytes_pending);

/** @brief Sets the second data byte and updates the state
 *  @param status_byte The status byte.
 *  @param pytes_pending True if more bytes are in the uart fifo.
 */
static void ICACHE_FLASH_ATTR midi_set_d2(uint8_t data_byte,
                                          uint8_t bytes_pending);

/** @brief Checks if the buffer is full.
 *  @details If the buffer is full, the transmit cb is called
 */
static void ICACHE_FLASH_ATTR midi_check_full_buffer();

void ICACHE_FLASH_ATTR midi_init(midi_done_cb_t cb) {
   int i;
   midi_data.state = MIDI_ST_INIT;
   midi_data.expected_data = 0;
   for (i = 0; i < MIDI_MSG_BUF_SIZE; i++) {
      midi_data.message[i] = 0;
   }
   midi_data.byte_cnt = 0;
   midi_data.cb = cb;
   midi_data.initialized = 1;
}

void ICACHE_FLASH_ATTR midi_add(uint8_t byte, uint8_t bytes_pending) {
   if (midi_data.initialized == 0) {
      return;
   }
   if (midi_data.state == MIDI_ST_INIT || midi_data.state == MIDI_ST_IDLE) {
      /* If we receive a non-state byte ... */
      if ((byte & 0x80) == 0) {
         if (midi_data.state == MIDI_ST_INIT) {
            /* skip it in INIT-state
             * because there is no 'last status'
             */
            return;
         } else {
            /* take is as first data byte and re-use the old status byte*/
            midi_set_d1(byte, bytes_pending);
            return;
         }
      } else {
         midi_set_status(byte, bytes_pending);
      }
   } else if (midi_data.state == MIDI_ST_WAIT_D1) {
      if ((byte & 0x80) == 0x80) {
         /* If there is a nother status byte,
          * use it and wait for the first data byte
          */
         midi_set_status(byte, bytes_pending);
         return;
      }
      midi_set_d1(byte, bytes_pending);

   } else if (midi_data.state == MIDI_ST_WAIT_D2) {
      if ((byte & 0x80) == 0x80) {
         /* If there is a status byte instead of a data byte,
          * use it and wait for the first data byte
          */
         midi_set_status(byte, bytes_pending);
         return;
      }
      midi_set_d2(byte, bytes_pending);
   } else if (midi_data.state == MIDI_ST_WAIT_F7) {
      midi_data.message[midi_data.byte_cnt++] = byte;

      if (byte == 0xf7) {
         /* End of message -> send the buffer */
         midi_data.cb(midi_data.message, midi_data.byte_cnt);
         midi_data.byte_cnt = 0;
         midi_data.state = MIDI_ST_IDLE;
      } else {
         midi_check_full_buffer();
      }
   }
}

static void ICACHE_FLASH_ATTR midi_set_status(uint8_t status_byte,
                                              uint8_t bytes_pending) {
   /* Handle status bytes with one data byte */
   if ((status_byte & 0xf0) == 0xC0 || /* Program Change */
       (status_byte & 0xf0) == 0xd0 || /* Channel Preassure / After touch */
       status_byte == 0xf1 ||          /* Time Code */
       status_byte == 0xf3) {          /* Song Select */
      midi_data.expected_data = 1;
      midi_data.message[midi_data.byte_cnt++] = status_byte;
      midi_data.state = MIDI_ST_WAIT_D1;
   }
   /* Handle status bytes with no data bytes */
   else if ((status_byte == 0xf4) || /* undefined */
            (status_byte == 0xf5) || /* undefined */
            (status_byte == 0xf6) || /* Tune Request */
            (status_byte == 0xf8) || /* Timing Clock */
            (status_byte == 0xf9) || /* Undefined */
            (status_byte == 0xfa) || /* Start */
            (status_byte == 0xfb) || /* Continue */
            (status_byte == 0xfc) || /* Stop */
            (status_byte == 0xfd) || /* Undefined */
            (status_byte == 0xfe) || /* Active Sensing */
            (status_byte == 0xff)) { /* System Reset */
      midi_data.message[midi_data.byte_cnt++] = status_byte;
      midi_data.state = MIDI_ST_IDLE;
      if (!bytes_pending) {
         midi_data.cb(midi_data.message, midi_data.byte_cnt);
         midi_data.byte_cnt = 0;
      }
   }
   /* Handle 0xf0 -> System Exclusive */
   else if (status_byte == 0xf0) {
      midi_data.expected_data = 0;
      midi_data.message[midi_data.byte_cnt++] = status_byte;
      midi_data.state = MIDI_ST_WAIT_F7;
   }
   /* For all other cases, 2 data bytes are expected:
    *   0x8n Note Off
    *   0x9n Note on
    *   0xan Poly Key Pressure
    *   0xbn Control Change
    *   0xen Pitch Bend Change
    *   0xf2 Song Position Counter
    * */
   else {
      midi_data.expected_data = 2;
      midi_data.message[midi_data.byte_cnt++] = status_byte;
      midi_data.state = MIDI_ST_WAIT_D1;
   }
   midi_check_full_buffer();
}

static void ICACHE_FLASH_ATTR midi_set_d1(uint8_t data_byte,
                                          uint8_t bytes_pending) {
   midi_data.message[midi_data.byte_cnt++] = data_byte;

   if (bytes_pending) {
      if (midi_data.expected_data == 1) {
         midi_data.state = MIDI_ST_IDLE;
      } else {
         midi_data.state = MIDI_ST_WAIT_D2;
      }
      midi_check_full_buffer();
   } else {
      if (midi_data.expected_data == 1) {
         midi_data.cb(midi_data.message, midi_data.byte_cnt);
         midi_data.byte_cnt = 0;
         midi_data.state = MIDI_ST_IDLE;
      } else {
         midi_data.state = MIDI_ST_WAIT_D2;
         midi_check_full_buffer();
      }
   }
}

static void ICACHE_FLASH_ATTR midi_set_d2(uint8_t data_byte,
                                          uint8_t bytes_pending) {
   midi_data.message[midi_data.byte_cnt++] = data_byte;

   midi_data.state = MIDI_ST_IDLE;
   if (!bytes_pending) {
      midi_data.cb(midi_data.message, midi_data.byte_cnt);
      midi_data.byte_cnt = 0;
   } else {
      midi_check_full_buffer();
   }
}

static void ICACHE_FLASH_ATTR midi_check_full_buffer() {
   /* Transmit the buffer if the buffer is full */
   if (midi_data.byte_cnt >= MIDI_MSG_BUF_SIZE) {
      midi_data.cb(midi_data.message, MIDI_MSG_BUF_SIZE);
      midi_data.byte_cnt = 0;
   }
}
