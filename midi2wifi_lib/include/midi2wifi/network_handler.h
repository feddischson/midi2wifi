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

#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define NETWORK_SERVER_SOCKET_RX_BUF 16

/** @brief Initializes the server socket.
 *  @param host_port Port at which the server socket is bind.
 *  @param tx_queue  See global tx_queue.
 *  @param rx_queue  See global rx_queue.
 */
int init_tcp_server_socket(
      int host_port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue );

int init_udp_socket(
      char* other_ip,
      int port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue );

/** @brief Initializes the client socket.
 *  @param host_ip   The ip address, to which the connection is established.
 *  @param host_port Port at which the host's server socket is listening.
 *  @param tx_queue  See global tx_queue.
 *  @param rx_queue  See global rx_queue.
 */
int init_tcp_client_socket(
      char* host_ip,
      int host_port,
      xQueueHandle tx_queue,
      xQueueHandle rx_queue );


/** @{
 *  @brief Network error codes
 */
#define NETWORK_SUCCESS              0
#define NETWORK_ERROR_SOCKET        -1
#define NETWORK_ERROR_BIND          -2
#define NETWORK_ERROR_SOCK_CLIENT   -3
#define NETWORK_ERROR_LISTEN        -4
#define NETWORK_ERROR_CONNECT       -5
/** @} */

#endif /* NETWORK_HANDLER_H */

