//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#include "tcpserver.h"
#include "tcperror.h"
#include "common/log.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Define substitute functions for MSVC compiler
#ifdef _MSC_VER
#include <WinSock2.h>
#include <Windows.h>
#pragma comment(lib, "Ws2_32.lib")
#define accept(s,a,l) WSAAccept(s,a,l,0,0)
#define close(s) closesocket(s)
#define socklen_t int32_t
#define ssize_t int
#define MSG_NOSIGNAL 0
#else
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#define SOCKADDR struct sockaddr_in
#define SOCKET int
#endif

// Chunk size when writing to socket
#define CHUNK_SIZE 65536

struct TcpServerState
{
    SOCKET listening_sockfd;
    SOCKET client_sockfd;
    _Bool connected;
    _Bool padding[3];
};

enum TcpErrorCode TcpServer_Initialise( uint16_t port, struct TcpServerState **new_state )
{
    struct TcpServerState *state = *new_state;
    struct sockaddr_in serv_addr;
#ifdef _MSC_VER
    char on = 1;
    unsigned long non_blocking = 1;
    WSADATA wsaData;
    if (WSAStartup( MAKEWORD( 2, 2 ), &wsaData ) != 0) return TCPERR_OPENING_SOCKET;
#else
    int on = 1, flags;
#endif

    if (state == NULL)
    {
        state = (struct TcpServerState *)malloc( sizeof( *state ) );
        if (state == NULL) return TCPERR_FAILED_MALLOC;
        memset( state, 0, sizeof( *state ) );
        *new_state = state;
    }

    Log( TRACE, LRC, "Initialising TCP server..." );

    state->listening_sockfd = socket( AF_INET, SOCK_STREAM, 0 );
    if (state->listening_sockfd < 0)
    {
        return TCPERR_OPENING_SOCKET;
    }

    // Enable address reuse
    if (setsockopt( state->listening_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof( on ) ) != 0)
    {
        TcpServer_Close( state );
        return TCPERR_SETTING_SOCKET_OPT;
    }

    // Disable Nagle algorithm that waits for additional data during read
    if (setsockopt( state->listening_sockfd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof( on ) ) != 0)
    {
        TcpServer_Close( state );
        return TCPERR_SETTING_SOCKET_OPT;
    }

    // Set the listening socket to non-blocking
#ifdef _MSC_VER
    if (ioctlsocket( state->listening_sockfd, FIONBIO, &non_blocking ))
    {
        TcpServer_Close( state );
        return TCPERR_SETTING_SOCKET_OPT;
    }
#else
    flags = fcntl( state->listening_sockfd, F_GETFL );
    if (flags < 0)
    {
        TcpServer_Close( state );
        return TCPERR_SETTING_SOCKET_OPT;
    }
    fcntl( state->listening_sockfd, F_SETFL, flags | O_NONBLOCK );
#endif

    // Bind all addresses to the socket
    memset( &serv_addr, 0, sizeof( serv_addr ) );
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( port );
    if (bind( state->listening_sockfd, (struct sockaddr *)&(serv_addr), sizeof( serv_addr ) ) < 0)
    {
        TcpServer_Close( state );
        return TCPERR_OPENING_SOCKET;
    }

    listen( state->listening_sockfd, 5 ); // Backlog (Queue length) = 5

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpServer_ClientConnect( struct TcpServerState *state, _Bool *connected )
{
    fd_set readfds;
    struct timeval timeout_struct = { 0, 0 };
    int waiting_connections;

    *connected = 0;

    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->listening_sockfd == 0) return TCPERR_OPENING_SOCKET;

    if (state->connected)
    {
        *connected = 1;
        return TCPERR_ALREADY_CONNECTED;
    }

    // See if there is a client trying to connect
    FD_ZERO( &readfds );
    FD_SET( state->listening_sockfd, &readfds );

    waiting_connections = select( (int)state->listening_sockfd + 1, &readfds, NULL, NULL, &timeout_struct );

    if (waiting_connections < 0)
    {
        return TCPERR_ACCEPTING_CONNECTION;
    }
    if (waiting_connections > 0)
    {
        // Accept the connection
        SOCKADDR cli_addr;
        socklen_t cli_len = sizeof( cli_addr );
        state->client_sockfd = accept( state->listening_sockfd, (struct sockaddr *)&cli_addr, &cli_len );

        if (state->client_sockfd < 0)
        {
            return TCPERR_ACCEPTING_CONNECTION;
        }

        state->connected = 1;
    }

    *connected = state->connected;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpServer_Fork( struct TcpServerState *state, struct TcpServerState **new_state )
{
    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected == 0) return TCPERR_NOT_CONNECTED;

    *new_state = (struct TcpServerState *)malloc( sizeof( *state ) );
    if (*new_state == NULL) return TCPERR_INVALID_STATE;
    memcpy( *new_state, state, sizeof( *state ) );

    (*new_state)->client_sockfd = 0;
    (*new_state)->connected = 0;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpServer_Disconnect( struct TcpServerState *state )
{
    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected == 0) return TCPERR_NOT_CONNECTED;

    close( state->client_sockfd );
    state->connected = 0;

    return TCPERR_NO_ERROR;
}

void TcpServer_Close( struct TcpServerState *state )
{
    if (state == NULL) return;

    if (state->client_sockfd)
    {
        close( state->client_sockfd );
    }
    if (state->listening_sockfd)
    {
        close( state->listening_sockfd );
    }
}

enum TcpErrorCode TcpServer_Get_Connected( struct TcpServerState *state, _Bool *connected )
{
    if (state == NULL) return TCPERR_INVALID_STATE;

    *connected = state->connected;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpServer_Read( struct TcpServerState *state, float64_t timeout, void *data, uint32_t max_length, uint32_t *length )
{
    fd_set readfds;
    struct timeval timeout_struct;
    int waiting_data = 1;
    ssize_t read_length;
    char *ptr = (char *)data;

    *length = 0;

    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected == 0) return TCPERR_NOT_CONNECTED;

    // See if there is data waiting to be read
    timeout_struct.tv_sec = (uint32_t)timeout;
    timeout_struct.tv_usec = (uint32_t)((timeout - timeout_struct.tv_sec) * 1e6);

    FD_ZERO( &readfds );
    FD_SET( state->client_sockfd, &readfds );

    while (waiting_data > 0 && *length < max_length)
    {
        waiting_data = select( (int)state->client_sockfd + 1, &readfds, NULL, NULL, &timeout_struct );

        // See if there is any data available
        if (waiting_data < 0)
        {
            TcpServer_Disconnect( state );
            return TCPERR_CONNECTION_LOST;
        }
        else if (waiting_data > 0)
        {
            // Read the available data
            read_length = recv( state->client_sockfd, ptr, max_length - *length, MSG_NOSIGNAL );
            if (read_length <= 0)
            {
                TcpServer_Disconnect( state );
                return TCPERR_CONNECTION_LOST;
            }

            ptr += read_length;
            *length += read_length;
        }
    }

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpServer_Write( struct TcpServerState *state, const void *data, uint32_t length )
{
    int write_length = 1;
    int remainder = (int)length;
    const char *ptr = (const char *)data;

    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected == 0) return TCPERR_NOT_CONNECTED;

    while (write_length > 0 && remainder > 0)
    {
        write_length = remainder > CHUNK_SIZE ? CHUNK_SIZE : remainder;

        // Write up to CHUNK_SIZE bytes at a time
        write_length = (int)send( state->client_sockfd, ptr, (size_t)write_length, MSG_NOSIGNAL );
        if (write_length == -1)
        {
            TcpServer_Disconnect( state );
            return TCPERR_CONNECTION_LOST;
        }

        ptr += write_length;
        remainder -= write_length;
    }
    if (remainder > 0)
    {
        return TCPERR_WRITING_TO_CLIENT;
    }

    return TCPERR_NO_ERROR;
}
