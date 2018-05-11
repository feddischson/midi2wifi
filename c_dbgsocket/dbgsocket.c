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


/* *
 *
 *  This is a small debug tool, which can be used to initate a
 *  TCP server or client socket.
 *
 *  Usage:
 *    ./dbgsocket 1 1234                  # Initiates a server socket,
 *                                          listening at port 1234
 *    ./dbgspcket 2 1234 "192.168.1.5     # Creates a client socket, and
 *                                          connects it to 192.168.1.5:1234
 *
 *  Note: This is just some hacky code used to test the communication
 *        of the midi2wifi firmware.
 *  */

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>

/** @brief Reads from a socket and dumps the result to stdout
 *  @param arg socket, must be cated to an int.
 */
void read_and_dump( void* arg );

/** @brief Creates server socket
 *  @param port_server The socket listening port
 * */
void server_socket( int port_server );


/** @brief Opens a client socket and connects to a server
 * @param port Port of the server socket
 * @param host_ip IP where the server socket is located
 * */
void client_socket( int port, char* host_ip );

#define TYPE_SERVER     1
#define TYPE_CLIENT     2


/** @brief Main-function
 *  @details
 *    Creates server-sockets, listen to it and creates
 *    an reader thread for each connection.
 **/
int main( int argc, char *argv[] ) {
   char *ptr;
   int type;
   int port_server;

   /* Ensure we get a second argument -> port-no */
   if( argc < 3 )
   {
      perror( "Wrong number of arguments, please provide port\n");
      exit(-1);
   }
   else
   {
      type        = strtol( argv[1], &ptr, 10 );
      port_server = strtol( argv[2], &ptr, 10 );
   }


   if( type == TYPE_SERVER )
   {
      server_socket( port_server );
   }
   else if( type == TYPE_CLIENT )
   {
      if( argc != 4 )
      {
         perror( "Wrong number of arguments, please provide also the host ip");
         exit(-1);
      }
      client_socket( port_server, argv[3]);

   }

}

void client_socket( int port, char* host_ip )
{
   int client_socket;
   int ret;
   struct sockaddr_in local_addr;
   struct sockaddr_in host_addr;

   /* Create a new socket ... */
   client_socket = socket(AF_INET, SOCK_STREAM, 0);
   if (client_socket < 0)
   {
      perror( "Failed to create socket" );
      exit(-1);
   }

   /* .. bind it ... */
   memset(&local_addr, 0, sizeof(local_addr));
   local_addr.sin_family = AF_INET;
   local_addr.sin_addr.s_addr = 0;
   local_addr.sin_port = htons(54627);
   ret = bind( client_socket,
               (struct sockaddr*)&local_addr,
               sizeof(local_addr) );
   if (ret)
   {
      perror( "Failed to bind socket");
      exit(-1);
   }

   /* .. and connect it */
   memset(&host_addr, 0, sizeof(host_addr));
   host_addr.sin_family = AF_INET;
   inet_aton( host_ip, &(host_addr.sin_addr.s_addr ) );
   host_addr.sin_port = htons(port);
   ret = connect(client_socket,
         (struct sockaddr*)&host_addr,
         sizeof(host_addr));
   if (ret) {
      close( client_socket );
      perror("Failed to connect");
      exit(-1);
   }
   while(1)
   {
      fd_set readfds;
      char buf[256];
      FD_ZERO( &readfds );
      FD_SET( STDIN_FILENO, &readfds );
      FD_SET( client_socket, &readfds );
      int max_sd;
      if( STDIN_FILENO > client_socket )
      {
         max_sd = STDIN_FILENO;
      }
      else
      {
         max_sd = client_socket;
      }

      int i = select( max_sd+1, &readfds , NULL , NULL , NULL);

      if( FD_ISSET( STDIN_FILENO, &readfds) )
      {
         int n = read( STDIN_FILENO, buf, 256 );
         if( n > 0 )
         {
            write( client_socket, buf, n );
         }
      }
      else if( FD_ISSET( client_socket, &readfds) )
      {
         int n;
         bzero( buf, 256 );
         n = read( client_socket, buf, 256 );
         printf("[%d] %s", client_socket, buf );
         fflush( stdout );
      }

   }

}


void server_socket( int port_server )
{
   int sock_server;
   int sock_client;
   int size_addr_client;
   struct sockaddr_in addr_server;
   struct sockaddr_in addr_client;

   /* Create and bind the socket */
   sock_server = socket(AF_INET, SOCK_STREAM, 0);
   if (sock_server < 0) {
      perror("ERROR opening socket");
      exit(1);
   }
   bzero((char *) &addr_server, sizeof(addr_server));
   addr_server.sin_family = AF_INET;
   addr_server.sin_addr.s_addr = INADDR_ANY;
   addr_server.sin_port = htons(port_server);
   if (bind(sock_server, (struct sockaddr *) &addr_server, sizeof(addr_server)) < 0) {
      perror("ERROR on binding");
      exit(1);
   }

   /* Listen for connections */
   listen(sock_server,5);
   size_addr_client = sizeof(addr_client);

   while (1)
   {

      /* accept incomming connections ... */
      sock_client = accept(sock_server, (struct sockaddr *) &addr_client, &size_addr_client);
      pthread_t read_thread;

      if (sock_client < 0) {
         perror( "ERROR on accept" );
         exit(-1);
      }
      printf( "[%d] Incomming connection\n", sock_server );

      /* ... and create a reader-thread */
      if( pthread_create( &read_thread , NULL ,  read_and_dump , (void*) sock_client) < 0)
      {
         perror( "Failed to create reader thread");
         exit(-1);
      }
   }
   return 0;
}


void read_and_dump( void *arg )
{
   int sock = (int) arg;
   int mode = 0;
   while(1)
   {
      int n;
      char buffer[256];
      bzero(buffer,256);
      n = read(sock,buffer,255);
      if (n < 0)
      {
         perror("ERROR reading from socket");
         return;
      }
      else
      {
         printf("[%d] %s", sock, buffer);
         fflush( stdout );
      }
   }
}
