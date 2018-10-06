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

#ifndef GLOBAL_H
#define GLOBAL_H


/* ****************************************************************** */

/** @brief If set to 1, network logging is enabled. */
#define ENABLE_NETWORK_LOGGING            0

/** @brief If set to 1, GPIO 2 is toggled every second. */
#define ENABLE_GPIO_TOGGLE                0

/** @brief Enables os_printf */
#define ENABLE_OS_PRINTF                  0

/** @brief If set to 1, the network logging info is also printed to UART */
#define ENABLE_NETWORK_LOGGING_TO_UART    0

/* ****************************************************************** */

/** @{
 * @brief Two modes are possible, the default is `infrastructure`
 */
/** @brief Uses an existing infrastructure */
#define NETWORK_MODE_INFRASTRUCTURE       0

/** @brief Opens an own wifi network */
#define NETWORK_MODE_OWN                  1

#ifndef NETWORK_MODE
 #define NETWORK_MODE NETWORK_MODE_INFRASTRUCTURE
#endif

#if   NETWORK_MODE==NETWORK_MODE_INFRASTRUCTURE
   #if !defined(NETWORK_INFRASTRUCTURE_SSID) || !defined(NETWORK_INFRASTRUCTURE_PASS)
      #error "No ssid and pass provided"
   #endif
#elif NETWORK_MODE==NETWORK_MODE_OWN
   #if !defined(MIDI2WIFI_SSID) ||  !defined(MIDI2WIFI_PASS)
      #error "No midi2wifi ssid and pass provided"
   #endif
#else
   #error "Unknown network mode"
#endif

/**  @}  */


/** @{
 * @brief Two firmware variants are possible, the default is `host`.
 */

/** @brief Host-variant
 *  @details
 *    Sets up wifi network and opens a server-socket,
 *    where the device connects to.
 */
#define MIDI_VARIANT_HOST     0

/** @brief Device-variant
 *  @details
 *    Tries to connect to the host network and socket with a client-socket.
 */
#define MIDI_VARIANT_DEVICE   1

#ifndef MIDI_VARIANT
   #define MIDI_VARIANT MIDI_VARIANT_HOST
#endif
/**  @}  */

/** @brief IP of the logging host. */
#define LOGGING_HOST       "192.168.222.11"

/** @brief TCP port of the logging host. */
#define LOGGING_PORT       58754

#if NETWORK_MODE==NETWORK_MODE_INFRASTRUCTURE
  /** @brief The host ip address. */
  #define HOST_IP_ADDRESS    "192.168.222.219"

  /** @brief The device ip address. */
  #define DEVICE_IP_ADDRESS   "192.168.222.215"

  /** @brief The host's server socket port */
  #define MIDI2WIFI_PORT          45678

#elif NETWORK_MODE==NETWORK_MODE_OWN

  /** @brief The host ip address. */
  #define HOST_IP_ADDRESS    "192.168.4.1"

  /** @brief The device ip address. */
  #define DEVICE_IP_ADDRESS   "192.168.4.2"

  /** @brief The host's server socket port */
  #define MIDI2WIFI_PORT          45678

#endif

/**
 * @{ 
 *   @brief Transport Layer definition and selection
 */
#define TRANSPORT_TCP 1
#define TRANSPORT_UDP 2
#define TRANSPORT TRANSPORT_UDP
/** 
 * @}
 */

#if MIDI_VARIANT==MIDI_VARIANT_HOST
   #define NAME         "MIDI2WIFI Adapter Host"
#elif MIDI_VARIANT==MIDI_VARIANT_DEVICE
   #define NAME         "MIDI2WIFI Adapter Device"
#else
   #error "Unknown midi variant"
#endif

#define RX_TX_QUEUE_SIZE   16

#define MIDI_UART_BAUD  31250
//#define MIDI_UART_BAUD 9600

extern void debug_toggle();
//#define DEBUG_TOGGLE dbg_toggle();
#define DEBUG_TOGGLE


#endif /* GLOBAL_H */

