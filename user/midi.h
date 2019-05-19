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
#ifndef __MIDI_H__
#define __MIDI_H__
#include <user_interface.h>

/** @brief Midi-done call-back definition.
 *  @details This callback function is called after
 *           receiving one complete midi message */
typedef int8_t (*midi_done_cb_t)(uint8_t *data, uint16_t len);

/** @brief Adds and processes one byte */
void ICACHE_FLASH_ATTR midi_add(uint8_t byte, uint8_t bytes_pending);

/** @brief Initializes the internal midi structure. */
void ICACHE_FLASH_ATTR midi_init(midi_done_cb_t cb);

#endif /* __MIDI_H__ */
