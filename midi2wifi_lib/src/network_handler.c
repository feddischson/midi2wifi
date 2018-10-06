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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "gpio.h"

#include "midi2wifi/network_handler.h"
#include "midi2wifi/global.h"
#include "midi2wifi/logging.h"


/** @brief Server-socket task, responsible for accepting connections.
 *  @param param Ignored.
 *  @return Nothing.
 */
void server_socket_task( void* param );

/** @brief TCP socket read task.
 *  @details
 *    Reads bytes from the tcp socket and writes them into the rx_queue.
 *  @param param Needs to be casted to an int, holds the socket.
 */
void socket_tcp_read_task( void* param );

/** @brief UDP socket read task.
 *  @details
 *    Reads bytes from the udp socket and writes them into the rx_queue.
 *  @param param Needs to be casted to an int, holds the socket.
 */
void socket_udp_read_task( void* param );

/** @brief TCP socket write task.
 *  @details
 *    Reads bytes from the tx_queue and writes them into tcp socket.
 *  @param param Needs to be casted to an int, holds the socket.
 */
void socket_tcp_write_task( void* param );

/** @brief TCP socket write task.
 *  @details
 *    Reads bytes from the tx_queue and writes them into tcp socket.
 *  @param param Needs to be casted to an int, holds the socket.
 */
void socket_udp_write_task( void* param );


/** @brief Private network data.
 */
static struct _network_data_
{
   int                  socket;      /**< Initialized by init_tcp_server_socket */
   struct sockaddr_in   addr_server; /**< Only used in host variant         */
   struct sockaddr_in   addr_other;

   xQueueHandle         tx_queue;    /**< See global tx_queue               */
   xQueueHandle         rx_queue;    /**< See global rx_queue               */
} network_data = {
   .socket = 0,
   .addr_server = {0},
   .addr_other  = {0},
   .tx_queue    = 0,
   .rx_queue    = 0
};


int init_tcp_client_socket(
      char* host_ip,
      int host_port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue )
{
   int ret;
   int socket;
   network_data.tx_queue   = tx_queue;
   network_data.rx_queue   = rx_queue;
   struct sockaddr_in local_addr;
   struct sockaddr_in host_addr;

   /* Create a new socket ... */
   socket = socket(AF_INET, SOCK_STREAM, 0);
   if (socket < 0)
   {
       return NETWORK_ERROR_SOCKET;
   }

   /* .. bind it ... */
   memset(&local_addr, 0, sizeof(local_addr));
   local_addr.sin_family = AF_INET;
   local_addr.sin_addr.s_addr = 0;
   local_addr.sin_port = htons(52628);
   ret = bind( socket,
               (struct sockaddr*)&local_addr,
               sizeof(local_addr) );
   if (ret)
   {
       return NETWORK_ERROR_BIND;
   }

   /* .. and connect it */
   memset(&host_addr, 0, sizeof(host_addr));
   host_addr.sin_family = AF_INET;
   inet_aton( host_ip, &(host_addr.sin_addr.s_addr ) );
   host_addr.sin_port = htons(host_port);
   ret = connect(socket,
         (struct sockaddr*)&host_addr,
         sizeof(host_addr));
   if (ret) {
      close( socket );
      return NETWORK_ERROR_CONNECT;
   }

   /* Start the read and write task */
   xTaskCreate( socket_tcp_read_task,  (signed char *)"TCP-Server-Read",
                  256, (void*)socket, configMAX_PRIORITIES-2, NULL);
   xTaskCreate( socket_tcp_write_task, (signed char *)"TCP-Server-Write",
                  256, (void*)socket, configMAX_PRIORITIES-2, NULL);

   return NETWORK_SUCCESS;
}


int init_udp_socket(
      char* other_ip,
      int port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue )
{
   network_data.tx_queue   = tx_queue;
   network_data.rx_queue   = rx_queue;

   /* Create the udp server socket ... */
   network_data.socket = socket(AF_INET, SOCK_DGRAM, 0);
   if (network_data.socket < 0)
   {
      return NETWORK_ERROR_SOCKET;
   }


   /* ... bind it ... */
   memset( (char*)&network_data.addr_server,
            0,
            sizeof(network_data.addr_server));
   network_data.addr_server.sin_family = AF_INET;
   network_data.addr_server.sin_addr.s_addr = INADDR_ANY;
   network_data.addr_server.sin_port = htons( port );


   memset( (char*)&network_data.addr_other,
            0,
            sizeof(network_data.addr_other));
   network_data.addr_other.sin_family = AF_INET;
   network_data.addr_other.sin_port = htons( port );
   network_data.addr_other.sin_addr.s_addr = INADDR_ANY;
   inet_aton( other_ip, &(network_data.addr_other.sin_addr.s_addr ) );


   /* ... bind it ... */
   if ( bind( network_data.socket,
            (struct sockaddr *) &network_data.addr_server,
             sizeof(network_data.addr_server)) < 0 )
   {
      return NETWORK_ERROR_BIND;
   }

   /* ... and if succeeded, start the read and write task */
   xTaskCreate( socket_udp_read_task,  (signed char *)"UDP-Read",
                  256, (void*)network_data.socket, configMAX_PRIORITIES-2, NULL);
   xTaskCreate( socket_udp_write_task, (signed char *)"UDP-Write",
                  256, (void*)network_data.socket, configMAX_PRIORITIES-2, NULL);

   return NETWORK_SUCCESS;
}



int init_tcp_server_socket(
      int host_port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue )
{

   network_data.tx_queue   = tx_queue;
   network_data.rx_queue   = rx_queue;

   /* Create the tcp server socket ... */
   network_data.socket = socket(AF_INET, SOCK_STREAM, 0);
   if (network_data.socket < 0)
   {
      return NETWORK_ERROR_SOCKET;
   }


   /* ... bind it ... */
   memset( (char*)&network_data.addr_server,
            0,
            sizeof(network_data.addr_server));
   network_data.addr_server.sin_family = AF_INET;
   network_data.addr_server.sin_addr.s_addr = INADDR_ANY;
   network_data.addr_server.sin_port = htons( host_port );
   if ( bind( network_data.socket,
            (struct sockaddr *) &network_data.addr_server,
             sizeof(network_data.addr_server)) < 0 )
   {
      return NETWORK_ERROR_BIND;
   }

   /* ... and listen to it. */
   if( listen(network_data.socket,1) != 0 )
   {
      return NETWORK_ERROR_LISTEN;
   }
   else
   {
      /* the server_socket_task is responsible for accepting the
       * connections */
      xTaskCreate( server_socket_task,
                   (signed char *)"TCP-Server",
                   256, NULL, configMAX_PRIORITIES-3, NULL );
      return NETWORK_SUCCESS;
   }

}

void server_socket_task( void* param )
{
   int sock_client;
   int size_addr_client;
   struct sockaddr_in addr_client;

   size_addr_client = sizeof(addr_client);

   while (1)
   {
      fd_set rfds;

      /* Accept a new connection ... */
      sock_client = accept(
            network_data.socket,
            (struct sockaddr *) &addr_client,
            &size_addr_client );
      LOG_TO_NETWORK("[NET] Incomming connection %d\n", sock_client);
      if( sock_client < 0 )
      {
         return;
      }

      /* ... and if succeeded, start the read and write task */
      xTaskCreate( socket_tcp_read_task,  (signed char *)"TCP-Server-Read",
                     256, (void*)sock_client, configMAX_PRIORITIES-2, NULL);
      xTaskCreate( socket_tcp_write_task, (signed char *)"TCP-Server-Write",
                     256, (void*)sock_client, configMAX_PRIORITIES-2, NULL);

   }
}

void socket_tcp_read_task( void* param )
{
   int n;
   char buffer[NETWORK_SERVER_SOCKET_RX_BUF];
   int sock_client = (int)(param);
   while(1)
   {
      int i;
      n = read(sock_client,buffer,NETWORK_SERVER_SOCKET_RX_BUF);
      DEBUG_TOGGLE
      if( n == 0 )
      {
         close( sock_client );
         vTaskSuspend( NULL );
      }
      else
      {
         for( i=0; i < n; i++ )
         {
            xQueueSend( network_data.rx_queue, &buffer[i], portMAX_DELAY );
         }
      }
   }
}

void socket_udp_read_task( void* param )
{
   int n;
   char buffer[NETWORK_SERVER_SOCKET_RX_BUF];
   struct sockaddr_in si_other;
   int sock = (int)(param);
   int slen;
   while(1)
   {
      int i;
      n = recvfrom(sock, buffer, NETWORK_SERVER_SOCKET_RX_BUF, 0, (struct sockaddr *)&si_other, &slen );
      DEBUG_TOGGLE
      if( n == 0 )
      {
         close( sock );
         vTaskSuspend( NULL );
      }
      else
      {
         for( i=0; i < n; i++ )
         {
            xQueueSend( network_data.rx_queue, &buffer[i], portMAX_DELAY );
         }
      }
   }
}

void socket_tcp_write_task( void* param )
{
   int n;
   uint8_t buf;
   int sock_client = (int)(param);
   while(1)
   {
      n = xQueueReceive(
            network_data.tx_queue,
            &buf,
            portMAX_DELAY
            );
      if( n )
      {
         DEBUG_TOGGLE
         n = write( sock_client, &buf, 1 );
         if( n == -1 )
         {
            close( sock_client );
            vTaskSuspend( NULL );
         }
      }
   }
}

void socket_udp_write_task( void* param )
{
   int n;
   uint8_t buf;
   int sock = (int)(param);

   while(1)
   {
      n = xQueueReceive(
            network_data.tx_queue,
            &buf,
            portMAX_DELAY
            );
      if( n )
      {
         DEBUG_TOGGLE
         n = sendto(sock, &buf, 1, 0, (struct sockaddr*) &network_data.addr_other, sizeof(network_data.addr_other));
         if( n == -1 )
         {
            close( sock );
            vTaskSuspend( NULL );
         }
      }
   }
}

