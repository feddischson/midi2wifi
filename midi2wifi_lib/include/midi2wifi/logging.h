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

#ifndef LOGGING_H
#define LOGGING_H

#include "midi2wifi/global.h"

/** @brief Connects the logging-socket
 *  @param host Host-ip address (as string)
 *  @param posr Host port
 *  @return 0 on success, otherwise a negative error number
 */
int log_connect_dbg_socket( const char * host, int port, const char* name );

/** @brief Logging function
 *  @details Writes the provided information to a buffer and then,
 *           writes the buffer to the socket.
 */
int log_to_network( const char* format, ... );


/** The log buffer size must be larger than the largest log-message */
#define LOG_BUF_SIZE 128


/** @{
 *  @brief Logging error codes
 *  */
#define LOG_SUCCESS            0
#define LOG_ERR_SOCK_CREATE   -1
#define LOG_ERR_BIND          -2
#define LOG_ERR_CONNECT       -3
#define LOG_ERR_FILE          -4
/** @} */

#if ENABLE_NETWORK_LOGGING && ! ENABLE_NETWORK_LOGGING_TO_UART
 #define LOG_TO_NETWORK(...) log_to_network( __VA_ARGS__ )
#else
 #define LOG_TO_NETWORK(...)
#endif

#endif // LOGGING_H

