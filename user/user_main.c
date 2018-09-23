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
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esp_common.h"
#include "gpio.h"
#include "uart.h"

#include "midi2wifi/wifi_state_machine.h"
#include "midi2wifi/logging.h"
#include "midi2wifi/uart_handler.h"

/** @brief Global Tx-queue.
 *  @details
 *    Data from the UART is written into this queue,
 *    and transfered to the socket.
 */
static xQueueHandle tx_queue;


/** @brief Global Tx-queue.
 *  @details
 *    Data from the socket is written into this queue,
 *    and transfered to the UART output.
 */
static xQueueHandle rx_queue;


/** @brief Is called when the software is connected to a WIFI network */
void station_connected_cb( void );

/** @brief Task which toggles GPIO-2 every 2 seconds.
 *  @details Is used for debugging purposes.
 *  @param param Not used.
 */
void GPIO_toggle_task ( void *param );

/** @brief Global WIFI initialization function
 *  @details
 *    Initializes the WIFI, depending on the global
 *    configuration.
 *  @param tx_queue See global tx_queue instance/
 *  @param rx_queue See global rx_queue instance/
 *  @returns Nothing.
 */
void init_wifi( xQueueHandle tx_queue,
                xQueueHandle rx_queue );


/**
 * @brief Required by the SDK.
 * @details
 *    SDK just reversed 4 sectors, used for rf init data and paramters.
 *    We add this function to force users to set rf cal sector, since
 *    we don't know which sector is free in user's application.
 *    sector map for last several sectors : ABCCC
 *       A : rf cal
 *       B : rf init data
 *       C : sdk parameters
 * @returns rf cal sector
 */
uint32_t user_rf_cal_sector_set( void )
{
    flash_size_map size_map = system_get_flash_size_map();
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

/** 
 * @brief Toggles GPIO2 for debugging purposes
 */
void dbg_toggle(void)
{
   GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<2);
   GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<2);
}


#if ENABLE_GPIO_TOGGLE
void GPIO_toggle_task (void *pvParameters)
{
   while(1)
   {
      // Delay and turn on
      vTaskDelay (1000/portTICK_RATE_MS);
      GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<2);

      // Delay and LED off
      vTaskDelay (1000/portTICK_RATE_MS);
      GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<2);
   }
}
#endif


void station_connected_cb( void )
{
   int res ;
   os_printf("STATION_CONNECT_CB\n\n");
   #if ENABLE_NETWORK_LOGGING
   res = log_connect_dbg_socket( LOGGING_HOST, LOGGING_PORT, NAME );

   // if we fail to connect ... try it once again
   if( res != LOG_SUCCESS )
   {
      os_printf("Failed to connect to network logging ... try it again ... ");
      res = log_connect_dbg_socket( LOGGING_HOST, LOGGING_PORT, NAME );
      if( res != LOG_SUCCESS )
      {
         os_printf( "failed again\n");
      }
      else
      {
         os_printf( "Succeeded\n");
      }
   }
   if( res == LOG_SUCCESS )
   {
      LOG_TO_NETWORK("SDK version:%s\n", system_get_sdk_version());
   }
   #endif

   #if MIDI_VARIANT==MIDI_VARIANT_DEVICE
      #if TRANSPORT==TRANSPORT_TCP
         init_tcp_client_socket(
            HOST_IP_ADDRESS,
            MIDI2WIFI_PORT,
            tx_queue,
            rx_queue );
      #elif TRANSPORT==TRANSPORT_UDP
         init_udp_socket(
            HOST_IP_ADDRESS,
            MIDI2WIFI_PORT,
            tx_queue,
            rx_queue );
      #endif
   #elif MIDI_VARIANT==MIDI_VARIANT_HOST
      #if NETWORK_MODE==NETWORK_MODE_INFRASTRUCTURE
         res = init_udp_socket(
            DEVICE_IP_ADDRESS,
            MIDI2WIFI_PORT,
            tx_queue,
            rx_queue );

         if(  res == 0 )
         {
            os_printf("initialized server socket successfully\n");
         }
         else
         {
            os_printf("Failed to intialize server socket: %d\n", res );
         }
      #endif
   #endif
}


void init_wifi( xQueueHandle tx_queue,
                xQueueHandle rx_queue )
{
   set_on_station_connect( &station_connected_cb );
   init_esp_wifi();

   #if NETWORK_MODE==NETWORK_MODE_INFRASTRUCTURE
   stop_wifi_ap();
   start_wifi_station(
      NETWORK_INFRASTRUCTURE_SSID,
      NETWORK_INFRASTRUCTURE_PASS );
   #elif NETWORK_MODE==NETWORK_MODE_OWN
      #if MIDI_VARIANT==MIDI_VARIANT_HOST
      stop_wifi_station();
      start_wifi_ap(
         MIDI2WIFI_SSID,
         MIDI2WIFI_PASS
            );
      {
         #if TRANSPORT == TRANSPORT_TCP
         int res = init_tcp_server_socket(
               MIDI2WIFI_PORT,
               tx_queue,
               rx_queue );
         #elif TRANSPORT == TRANSPORT_UDP
         int res = init_udp_socket(
            DEVICE_IP_ADDRESS,
            MIDI2WIFI_PORT,
            tx_queue,
            rx_queue );
         #endif
         if(  res == 0 )
         {
            os_printf("initialized server socket successfully\n");
         }
         else
         {
            os_printf("Failed to intialize server socket: %d\n", res );
         }
      }
      #elif MIDI_VARIANT==MIDI_VARIANT_DEVICE
      // conenct to own network
      stop_wifi_ap();
      start_wifi_station(
         MIDI2WIFI_SSID,
         MIDI2WIFI_PASS
            );
      #endif
   #endif


}

/** @brief Dummy function to disable printing to uart
 *  @param c Ignored.
 **/
void user_printf(char c)
{
   // do nothing here
}

/** @brief Main-entry for the user-application, called by the SDK.
 */
void user_init(void)
{
   tx_queue = xQueueCreate( RX_TX_QUEUE_SIZE, sizeof(portCHAR) );
   rx_queue = xQueueCreate( RX_TX_QUEUE_SIZE, sizeof(portCHAR) );



   uart_init( UART0, MIDI_UART_BAUD, tx_queue, rx_queue );

   #if !ENABLE_OS_PRINTF
   os_install_putc1( user_printf );
   #endif

   init_wifi( tx_queue, rx_queue );

   /* 
    * Set GPIO2 as output and to 0 (low)
    */
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO2);
   PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);
   gpio_output_set(0, BIT2, BIT2, 0);

   #if ENABLE_GPIO_TOGGLE
   xTaskCreate( GPIO_toggle_task, (signed char *)"Blink", 256, NULL, 2, NULL );
   #endif
   log_to_network( "init done\n" );

   os_printf("init done\n");
}

