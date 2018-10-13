/* ********************************************************************
 *
 *  Copyright (c) 2018 Christian Haettich [feddischson@gmail.com]
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

struct _midi_data {
   uint8_t initialized;   /**< Init-flag: 1=struct is initialized */
   uint8_t state;         /**< Current processing state */
   uint8_t expected_data; /**< Length of the expected data */
   uint8_t message[3];    /**< Message buffer */
   midi_done_cb_t cb;     /**< Call-back function */
} midi_data = {0, MIDI_ST_INIT, 0, {0}, 0};

/** @brief Sets the first byte and updates the state */
static void ICACHE_FLASH_ATTR midi_set_status(uint8_t status_byte);

/** @brief Sets the first data byte and updates the state */
static void ICACHE_FLASH_ATTR midi_set_d1(uint8_t data_byte);

/** @brief Sets the second data byte and updates the state */
static void ICACHE_FLASH_ATTR midi_set_d2(uint8_t data_byte);

void ICACHE_FLASH_ATTR midi_init(midi_done_cb_t cb) {
   midi_data.state = MIDI_ST_INIT;
   midi_data.expected_data = 0;
   midi_data.message[0] = 0;
   midi_data.message[1] = 0;
   midi_data.message[2] = 0;
   midi_data.cb = cb;
   midi_data.initialized = 1;
}

void ICACHE_FLASH_ATTR midi_add(uint8_t byte) {
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
            midi_set_d1(byte);
            return;
         }
      }
      midi_set_status(byte);
   }

   else if (midi_data.state == MIDI_ST_WAIT_D1) {
      if ((byte & 0x80) == 0x80) {
         /* If there is a nother status byte,
          * use it and wait for the first data byte
          */
         midi_set_status(byte);
         return;
      }
      midi_set_d1(byte);

   } else if (midi_data.state == MIDI_ST_WAIT_D2) {
      if ((byte & 0x80) == 0x80) {
         /* If there is a status byte instead of a data byte,
          * use it and wait for the first data byte
          */
         midi_set_status(byte);
         return;
      }
      midi_set_d2(byte);
   }
}

static void ICACHE_FLASH_ATTR midi_set_status(uint8_t status_byte) {
   if ((status_byte & 0xf0) == 0xC0 || (status_byte & 0xf0) == 0xD0 ||
       (status_byte & 0xf0) == 0xF0) {
      midi_data.expected_data = 1;
   } else {
      midi_data.expected_data = 2;
   }

   midi_data.message[0] = status_byte;
   midi_data.state = MIDI_ST_WAIT_D1;
}

static void ICACHE_FLASH_ATTR midi_set_d1(uint8_t data_byte) {
   midi_data.message[1] = data_byte;
   if (midi_data.expected_data == 1) {
      midi_data.cb(midi_data.message, 2);
      midi_data.state = MIDI_ST_IDLE;
   } else {
      midi_data.state = MIDI_ST_WAIT_D2;
   }
}

static void ICACHE_FLASH_ATTR midi_set_d2(uint8_t data_byte) {
   midi_data.message[2] = data_byte;
   midi_data.cb(midi_data.message, 3);
   midi_data.state = MIDI_ST_IDLE;
}
