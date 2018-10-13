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
#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/** @{
 * @brief Two firmware variants are possible, the default is `host`.
 */

/** @brief Host-variant
 *  @details
 *    Sets up wifi network and opens a server-socket,
 *    where the device connects to.
 */
#define M2W_VARIANT_HOST 0

/** @brief Device-variant
 *  @details
 *    Tries to connect to the host network and socket with a client-socket.
 */
#define M2W_VARIANT_DEVICE 1

#ifndef M2W_VARIANT
#define M2W_VARIANT M2W_VARIANT_HOST
#endif

#if M2W_VARIANT != M2W_VARIANT_HOST && M2W_VARIANT != M2W_VARIANT_DEVICE
#error Unsupported Variant
#endif
/**  @}  */

#ifndef M2W_UART_BAUD
#define M2W_UART_BAUD 31250
#endif

#ifndef M2W_SSID
#define M2W_SSID "MIDI2WIFI"
#endif

#ifndef M2W_KEY
#define M2W_KEY "ApX8t6H3ounXdckpKRo9fAtEqTwcWhu"
#endif

#ifndef M2W_WIFI_CHANNEL
#define M2W_WIFI_CHANNEL 13
#endif

/** @{
 *  @brief Internal defines
 * */

/** @brief Required in order to have a smaller timer interval */
#define USE_US_TIMER 1

#define ENABLE_OS_PRINTF 0

#define M2W_UDP_PORT_1 8899
#define M2W_UDP_PORT_2 9988

/** @brief Used to do some timing measurements */
#define M2W_DEBUG_IO_TOGGLE 0

/** @brief No auth. if set to 1, otherwise WPA2_PSK */
#define M2W_OPEN_WIFI 0

/** @brief The internal UART buffer size */
#define M2W_UART_BUF_SIZE 8

/** @brief If set to 1, the SSID is hidden */
#define M2W_HIDDEN_SSID 1


/** @}  */

#endif
