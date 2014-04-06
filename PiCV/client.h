/*
 * client.h
 *
 *  Created on: 19 nov. 2013
 *      Author: Jean
 */

#ifndef CLIENT_H_
#define CLIENT_H_

#if defined (WIN32)
    #include <winsock2.h>
    typedef int socklen_t;
#elif defined (linux)
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket(s) close(s)
    typedef int SOCKET;
    typedef struct sockaddr_in SOCKADDR_IN;
    typedef struct sockaddr SOCKADDR;
#endif

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <stddef.h>
#include <string.h>
#include <math.h>

#define PORT 23
#define LENGTH_BUFFER 80
#define NB_PROD 1000

int init_client(socket *sock, char data);

#endif /* CLIENT_H_ */
