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

#include <stdarg.h>
#include <stdio.h>

#include "esp_common.h"
#include "lwip/sockets.h"

#include "midi2wifi/logging.h"


static struct _loging_data_
{
   int socket;
   struct sockaddr_in local_addr;
   struct sockaddr_in host_addr;

} log_data = {
   .socket = -1,
   .local_addr = {0},
   .host_addr  = {0}
};

int log_to_network( const char* format, ... )
{
   va_list arglist;
   if( log_data.socket < 0 )
   {
      return 0;
   }
   else
   {
      int res;
      char buf[LOG_BUF_SIZE];
      bzero( buf, LOG_BUF_SIZE );
      va_start( arglist, format );
      res = vsnprintf( buf, sizeof( buf ), format, arglist );
      va_end( arglist );
      write( log_data.socket, buf, res );

      #if ENABLE_NETWORK_LOGGING_TO_UART
      os_printf( "%s", buf );
      #endif
      return res;
   }
}

int log_connect_dbg_socket( const char * host_ip, int port, const char* name )
{
   int ret;

   /* close the socket if open */
   if( log_data.socket  >= 0 )
   {
      close( log_data.socket );
   }

   /* Create a new socket ... */
   log_data.socket = socket(AF_INET, SOCK_STREAM, 0);
   if (log_data.socket < 0)
   {
       return LOG_ERR_SOCK_CREATE;
   }

   /* .. bind it ... */
   memset(&log_data.local_addr, 0, sizeof(log_data.local_addr));
   log_data.local_addr.sin_family = AF_INET;
   log_data.local_addr.sin_addr.s_addr = 0;
   log_data.local_addr.sin_port = htons(54622);
   ret = bind( log_data.socket,
               (struct sockaddr*)&log_data.local_addr,
               sizeof(log_data.local_addr) );
   if (ret)
   {
       return LOG_ERR_BIND;
   }

   /* .. and connect it */
   memset(&log_data.host_addr, 0, sizeof(log_data.host_addr));
   log_data.host_addr.sin_family = AF_INET;
   inet_aton( host_ip, &(log_data.host_addr.sin_addr.s_addr ) );
   log_data.host_addr.sin_port = htons(port);
   ret = connect(log_data.socket,
         (struct sockaddr*)&log_data.host_addr,
         sizeof(log_data.host_addr));
   if (ret) {
      close( log_data.socket );
      return LOG_ERR_CONNECT;
   }

   log_to_network( "______________________\n" );
   log_to_network( "hello from %s \n", name );
   return LOG_SUCCESS;
}

