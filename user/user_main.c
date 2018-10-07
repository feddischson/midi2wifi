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
#include <osapi.h>

#include <driver/uart.h>
#include <driver/uart_register.h>

#include <ip_addr.h>
/* must be after id_addr.h */
#include <espconn.h>

#include <user_interface.h>

/* *********************************
 *    Function Prototypes
 * */

static struct espconn conn1;
static struct espconn conn2;
static esp_udp udp1;
static esp_udp udp2;
static uint8_t connected;
static ETSTimer uart_rx_poll_timer;

static void ICACHE_FLASH_ATTR uart_rx_task();

static void ICACHE_FLASH_ATTR wifi_init_ap(char *ssid, char *key,
                                           uint8_t channel);
static void ICACHE_FLASH_ATTR wifi_event_cb(System_Event_t *evt);
static void ICACHE_FLASH_ATTR wifi_connect_to_ap(char *ssid, char *key);

static int8_t ICACHE_FLASH_ATTR init_udp_rx(void);
static void ICACHE_FLASH_ATTR udp_rx_cb(void *arg, char *pdata,
                                        unsigned short len);
static int8_t ICACHE_FLASH_ATTR udp_tx(uint8_t *data, uint16_t size);

static void ICACHE_FLASH_ATTR user_printf(char c);
static void ICACHE_FLASH_ATTR fatal_error(char *msg, uint32_t line);

void ICACHE_FLASH_ATTR user_init(void);

static inline void dbg_toggle(void) {
   GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << 2);
   GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << 2);
}
#if M2W_DEBUG_IO_TOGGLE
#define DBG_TOGGLE dbg_toggle()
#else
#define DBG_TOGGLE
#endif

#define FATAL_ERROR(_msg_) fatal_error(_msg_, __LINE__)

#define MIDI_ST_INIT 0
#define MIDI_ST_IDLE 1
#define MIDI_ST_WAIT_D1 2
#define MIDI_ST_WAIT_D2 3
struct _midi_data {
   uint8_t status;
   uint8_t expected_data;
   uint8_t message[3];
} midi_data = {MIDI_ST_INIT, 0, {0}};
static void midi_set_status(uint8_t status_byte);
static void midi_set_d1(uint8_t data_byte);
static void midi_set_d2(uint8_t data_byte);

/* ********************************* */

/** @brief Main entry point:
 *  @details Initializes
 *   - GPIO
 *   - UART (timer)
 *   - WIFI
 */
void ICACHE_FLASH_ATTR user_init(void) {
   /* re-init system timer to get us timers */
   system_timer_reinit();

   /* init the gpio driver
    * and set GPIO2 as output and to 0 (low)
    */
   gpio_init();
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
   PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);
   GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, BIT2);
   GPIO_OUTPUT_SET(GPIO_ID_PIN(2), 0);

   /* init UART */
   uart_init(M2W_UART_BAUD, M2W_UART_BAUD);
#if ENABLE_OS_PRINTF == 0
   os_install_putc1(user_printf);
#endif

#if M2W_VARIANT == M2W_VARIANT_HOST
   connected = 1;
   wifi_init_ap(M2W_SSID, M2W_KEY, M2W_WIFI_CHANNEL);
#elif M2W_VARIANT == M2W_VARIANT_DEVICE
   connected = 0;
   wifi_connect_to_ap(M2W_SSID, M2W_KEY);
#endif
   if (init_udp_rx() != 0) {
      FATAL_ERROR("Failed to initialize UDP RX");
   }

   /* init timer */
   os_timer_disarm(&uart_rx_poll_timer);
   os_timer_setfn(&uart_rx_poll_timer, (os_timer_func_t *)uart_rx_task, NULL);
   os_timer_arm_us(&uart_rx_poll_timer, 4, 0);
}

/** @brief Dummy function to disable printing to uart
 *  @param c Ignored.
 **/
static void ICACHE_FLASH_ATTR user_printf(char c) { /*nothing todo*/
}

/** @brief Returns never after a fatal error.
 *  @details
 *    After a while, the watch dog will do a restart.
 */
static void ICACHE_FLASH_ATTR fatal_error(char *msg, uint32_t line) {
   os_printf("fatal_error (%d):%s\n", line, msg);
   while (1)
      ;
}

/** @brief Sets MIDI status byte in internal structure
 * */
static void midi_set_status(uint8_t status_byte) {
   if ((status_byte & 0xf0) == 0xC0 || (status_byte & 0xf0) == 0xD0 ||
       (status_byte & 0xf0) == 0xF0) {
      midi_data.expected_data = 1;
   } else {
      midi_data.expected_data = 2;
   }

   midi_data.message[0] = status_byte;
   midi_data.status = MIDI_ST_WAIT_D1;
}

/** @brief Sets MIDI status data-byte 1 in internal structure
 * */
static void midi_set_d1(uint8_t data_byte) {
   midi_data.message[1] = data_byte;
   if (midi_data.expected_data == 1) {
      DBG_TOGGLE;
      udp_tx(midi_data.message, 2);
      midi_data.status = MIDI_ST_IDLE;
   } else {
      midi_data.status = MIDI_ST_WAIT_D2;
   }
}

/** @brief Sets MIDI status data-byte 2 in internal structure
 * */
static void midi_set_d2(uint8_t data_byte) {
   midi_data.message[2] = data_byte;
   DBG_TOGGLE;
   udp_tx(midi_data.message, 3);
   midi_data.status = MIDI_ST_IDLE;
}

/** @brief Process serial midi stream
 * */
uint8_t process_midi(uint8_t *data, uint16_t len) {
   uint16_t i_byte;

   for (i_byte = 0; i_byte < len; i_byte++) {
      uint8_t byte = data[i_byte];

      if (midi_data.status == MIDI_ST_INIT ||
          midi_data.status == MIDI_ST_IDLE) {
         /* If we receive a non-status byte ... */
         if (byte & 0x80 == 0) {
            if (midi_data.status == MIDI_ST_INIT) {
               /* skip it in INIT-state
                * because there is no 'last status'
                */
               continue;
            } else {
               /* take is as first data byte and re-use the old status byte*/
               midi_set_d1(byte);
               continue;
            }
         }
         midi_set_status(byte);
      }

      else if (midi_data.status == MIDI_ST_WAIT_D1) {
         if (byte & 0x80 == 1) {
            /* If there is a nother status byte,
             * use it and wait for the first data byte
             */
            midi_set_status(byte);
            continue;
         }
         midi_set_d1(byte);

      } else if (midi_data.status == MIDI_ST_WAIT_D2) {
         if (byte & 0x80 == 1) {
            /* If there is a status byte instead of a data byte,
             * use it and wait for the first data byte
             */
            midi_set_status(byte);
            continue;
         }
         midi_set_d2(byte);
      }
   }
}

/** @brief UART RX polling task.
 *  @details
 *   Polls the uart rx buffer via rx_buff_deq().
 *   If bytes are available, they are send via udp_tx().
 */
static void ICACHE_FLASH_ATTR uart_rx_task() {
   char uart_buf[M2W_UART_BUF_SIZE] = {0};
   uint16_t len = 0;
   os_timer_disarm(&uart_rx_poll_timer);
   len = rx_buff_deq(uart_buf, M2W_UART_BUF_SIZE);

#if M2W_SKIP_MIDI_INTERPRETATION == 1
   if (len > 0 && connected) {
      DBG_TOGGLE;
      if (udp_tx((uint8_t *)uart_buf, len) != 0) {
         udp_tx((uint8_t *)uart_buf, len);
      }
   }
#else
   if (len > 0 && connected) {
      process_midi((uint8_t *)uart_buf, len);
   }
#endif
   os_timer_arm_us(&uart_rx_poll_timer, 4, 0);
}

/** @brief Initializes the wifi access point
 *  @param ssid Name of the access point
 *  @param key Key for the authentication
 *  @param channel WIFI channel number
 *  @return Nothing.
 */
static void ICACHE_FLASH_ATTR wifi_init_ap(char *ssid, char *key,
                                           uint8_t channel) {
   static struct softap_config apconf;
   wifi_set_opmode(SOFTAP_MODE);
   wifi_softap_get_config(&apconf);

   os_memset(apconf.ssid, 0, 32);
   os_memset(apconf.password, 0, 64);
   os_strncpy((char *)apconf.ssid, ssid, 32);
   os_strncpy((char *)apconf.password, key, 64);
#if M2W_OPEN_WIFI == 1
   apconf.authmode = AUTH_OPEN;
#else
   apconf.authmode = AUTH_WPA2_PSK;
#endif
   apconf.max_connection = 4;
   apconf.ssid_hidden = M2W_HIDDEN_SSID;
   apconf.ssid_len = os_strlen(ssid);
   apconf.channel = channel;

   wifi_softap_set_config(&apconf);
}

/** @brief Is called for each wifi event
 *  @details
 *   Is only used in the DEVICE variant, where the
 *   connected flag is set and cleared.
 */
static void ICACHE_FLASH_ATTR wifi_event_cb(System_Event_t *evt) {
#if M2W_VARIANT == M2W_VARIANT_DEVICE
   if (evt->event == EVENT_STAMODE_GOT_IP) {
      connected = 1;
   } else if (evt->event == EVENT_STAMODE_DISCONNECTED) {
      connected = 0;
   }
#endif
}

/** @brief Initiates the connection to a WIFI access point.
 *  @param ssid Name of the access point
 *  @param key Key for the authentication
 *  @return Nothing.
 */
static void ICACHE_FLASH_ATTR wifi_connect_to_ap(char *ssid, char *key) {
   wifi_set_phy_mode(PHY_MODE_11N);
   struct station_config apconf;
   wifi_station_get_config(&apconf);

   os_memset(apconf.ssid, 0, 32);
   os_memset(apconf.password, 0, 64);

   os_strncpy((char *)apconf.ssid, ssid, 32);
   os_strncpy((char *)apconf.password, key, 64);
   apconf.bssid_set = 0;

   wifi_station_set_config(&apconf);
   wifi_set_event_handler_cb(wifi_event_cb);
   wifi_station_set_auto_connect(true);
   wifi_set_opmode(STATION_MODE);
}

/** @brief Initializes the UDP rx functionality */
static int8_t ICACHE_FLASH_ATTR init_udp_rx(void) {
   int8_t err;
   conn1.type = ESPCONN_UDP;
   conn1.state = ESPCONN_NONE;
#if M2W_VARIANT == M2W_VARIANT_HOST
   udp1.local_port = M2W_UDP_PORT_2;
#elif M2W_VARIANT == M2W_VARIANT_DEVICE
   udp1.local_port = M2W_UDP_PORT_1;
#endif
   conn1.proto.udp = &udp1;
   if ((err = espconn_create(&conn1)) != 0) {
      return err;
   }

   if ((err = espconn_regist_recvcb(&conn1, &udp_rx_cb)) != 0) {
      return err;
   }
   return 0;
}

/** @brief The UDP rx callback function */
static void ICACHE_FLASH_ATTR udp_rx_cb(void *arg, char *pdata,
                                        unsigned short len) {
   DBG_TOGGLE;
   unsigned short i;
   for (i = 0; i < len; i++) {
      uart_tx_one_char(0, pdata[i]);
   }
}

/** @brief The UDP TX function
 *  @param data Data payload
 *  @param size Size of the data payload
 *  @return 0 if there is no error, otherwise > 0.
 */
static int8_t ICACHE_FLASH_ATTR udp_tx(uint8_t *data, uint16_t size) {
   int8_t err;
   conn2.type = ESPCONN_UDP;
   conn2.state = ESPCONN_NONE;
   conn2.proto.udp = &udp2;
#if M2W_VARIANT == M2W_VARIANT_HOST
   IP4_ADDR((ip_addr_t *)conn2.proto.udp->remote_ip, 192, 168, 4, 2);
   conn2.proto.udp->local_port = espconn_port();
   conn2.proto.udp->remote_port = M2W_UDP_PORT_1;
#elif M2W_VARIANT == M2W_VARIANT_DEVICE
   IP4_ADDR((ip_addr_t *)conn2.proto.udp->remote_ip, 192, 168, 4, 1);
   conn2.proto.udp->local_port = espconn_port();
   conn2.proto.udp->remote_port = M2W_UDP_PORT_2;
#endif

   if ((err = espconn_create(&conn2)) != 0) {
      return err;
   }

   /* if sending failes, try to delete the connection
    * and return error code */
   if ((err = espconn_sent(&conn2, data, size)) != 0) {
      espconn_delete(&conn2);
      return err;
   }

   /* the API says, that nothign is returned */
   if ((err = espconn_delete(&conn2)) != 0) {
      /* if something != 0 is returned -> fatal-error */
      FATAL_ERROR("did not expect a return value other than 0");
   }
   return 0;
}

/** @brief Required to reserve the rf cal sector */
uint32_t ICACHE_FLASH_ATTR user_rf_cal_sector_set(void) {
   enum flash_size_map size_map = system_get_flash_size_map();
   uint32_t rf_cal_sec = 0;

   switch (size_map) {
      case FLASH_SIZE_4M_MAP_256_256:
         rf_cal_sec = 128 - 5;
         break;

      case FLASH_SIZE_8M_MAP_512_512:
         rf_cal_sec = 256 - 5;
         break;

      case FLASH_SIZE_16M_MAP_512_512:
      case FLASH_SIZE_16M_MAP_1024_1024:
         rf_cal_sec = 512 - 5;
         break;

      case FLASH_SIZE_32M_MAP_512_512:
      case FLASH_SIZE_32M_MAP_1024_1024:
         rf_cal_sec = 1024 - 5;
         break;

      case FLASH_SIZE_64M_MAP_1024_1024:
         rf_cal_sec = 2048 - 5;
         break;
      case FLASH_SIZE_128M_MAP_1024_1024:
         rf_cal_sec = 4096 - 5;
         break;
      default:
         rf_cal_sec = 0;
         break;
   }
   return rf_cal_sec;
}
