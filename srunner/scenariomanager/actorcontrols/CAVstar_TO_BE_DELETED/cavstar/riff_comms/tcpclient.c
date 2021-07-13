//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#include "tcpclient.h"
#include "tcperror.h"
#include "common/float.h"
#include "common/log.h"
#include "common/time.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Define substitute functions for MSVC compiler
#ifdef _MSC_VER
#include <Windows.h>
#pragma comment(lib, "Ws2_32.lib")
#define close(s) closesocket(s)
#define socklen_t int32_t
#define FLAG char
#define ssize_t int
#define MSG_NOSIGNAL 0
#else
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <arpa/inet.h>
#define FLAG int
#define SOCKET int
#endif

// Chunk size when writing to socket
#define CHUNK_SIZE 65536

struct TcpClientState
{
    char *hostname;
    uint16_t port;
    _Bool connected;
    _Bool padding;

    SOCKET client_sockfd;
    struct sockaddr_in sockaddrin;
    float64_t connect_time;
};

static enum TcpErrorCode TcpClient_Set_Non_Blocking( struct TcpClientState *state, uint32_t non_blocking );

enum TcpErrorCode TcpClient_Initialise( char *hostname, uint16_t port, struct TcpClientState **new_state )
{
    struct TcpClientState *state = *new_state;
#ifdef _MSC_VER
    WSADATA wsaData;
    if (WSAStartup( MAKEWORD( 2, 2 ), &wsaData ) != 0) return TCPERR_OPENING_SOCKET;
#endif

    if (state == NULL)
    {
        state = (struct TcpClientState *)malloc( sizeof( *state ) );
        if (state == NULL) return TCPERR_FAILED_MALLOC;
        memset( state, 0, sizeof( *state ) );
        *new_state = state;
    }

    Log( TRACE, LRC, "Initialising TCP client..." );

    if (state->hostname) free( state->hostname );
    state->hostname = (char *)malloc( strlen( hostname ) + 1 );
    if (state->hostname) memcpy( state->hostname, hostname, strlen( hostname ) + 1 );

    state->port = port;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Connect( struct TcpClientState *state, float64_t timeout, _Bool *connected )
{
    FLAG on = 1;
    fd_set writefds;
    struct timeval timeout_struct = { 0, 0 };

    *connected = 0;

    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected)
    {
        *connected = 1;
        return TCPERR_ALREADY_CONNECTED;
    }

    if (state->connect_time == 0.0)
    {
        state->client_sockfd = socket( AF_INET, SOCK_STREAM, 0 );
        if (state->client_sockfd < 0)
        {
            return TCPERR_OPENING_SOCKET;
        }

        // Disable Nagle algorithm that waits for additional data during read
        if (setsockopt( state->client_sockfd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof( on ) ) != 0)
        {
            return TCPERR_SETTING_SOCKET_OPT;
        }

        // Prepare the socket address and port
        memset( state->sockaddrin.sin_zero, '\0', sizeof( state->sockaddrin.sin_zero ) );
        state->sockaddrin.sin_family = AF_INET;
        state->sockaddrin.sin_addr.s_addr = inet_addr( state->hostname );
        state->sockaddrin.sin_port = htons( state->port );

        enum TcpErrorCode return_code = TcpClient_Set_Non_Blocking( state, 1 );
        if (return_code) return return_code;

        connect( state->client_sockfd, (struct sockaddr *)(&state->sockaddrin), sizeof( struct sockaddr_in ) );

        state->connect_time = Time_Get_Monotonic();
    }

    // See if the connection is successful
    FD_ZERO( &writefds );
    FD_SET( state->client_sockfd, &writefds );

    if (select( (int)state->client_sockfd + 1, NULL, &writefds, NULL, &timeout_struct ) > 0)
    {
        int result;
        socklen_t result_len = sizeof( result );
        if (getsockopt( state->client_sockfd, SOL_SOCKET, SO_ERROR, (FLAG *)&result, &result_len ) < 0)
        {
            return TCPERR_SETTING_SOCKET_OPT;
        }

        if (result == 0)
        {
            enum TcpErrorCode return_code = TcpClient_Set_Non_Blocking( state, 0 );
            if (return_code) return return_code;

            state->connected = 1;
        }
        else
        {
            close( state->client_sockfd );
            state->connect_time = 0.0;
        }
    }
    else if (Time_Get_Monotonic() > (state->connect_time + timeout))
    {
        close( state->client_sockfd );
        state->connect_time = 0.0;
    }

    *connected = state->connected;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Set_Non_Blocking( struct TcpClientState *state, uint32_t non_blocking )
{
    if (state == NULL) return TCPERR_INVALID_STATE;
#ifdef _MSC_VER
    if (ioctlsocket( state->client_sockfd, FIONBIO, &non_blocking ))
    {
        return TCPERR_SETTING_SOCKET_OPT;
    }
#else
    int flags = fcntl( state->client_sockfd, F_GETFL );
    if (flags < 0)
    {
        return TCPERR_SETTING_SOCKET_OPT;
    }
    flags &= ~O_NONBLOCK;
    if (non_blocking)
    {
        flags |= O_NONBLOCK;
    }
    fcntl( state->client_sockfd, F_SETFL, flags );
#endif
    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Disconnect( struct TcpClientState *state )
{
    if (state == NULL) return TCPERR_INVALID_STATE;

    if (state->connected == 0) return TCPERR_NOT_CONNECTED;

    close( state->client_sockfd );
    state->connected = 0;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Get_Connected( struct TcpClientState *state, _Bool *connected )
{
    if (state == NULL) return TCPERR_INVALID_STATE;

    *connected = state->connected;

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Read( struct TcpClientState *state, float64_t timeout, void *data, uint32_t max_length, uint32_t *length )
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
            TcpClient_Disconnect( state );
            return TCPERR_CONNECTION_LOST;
        }
        else if (waiting_data > 0)
        {
            // Read the available data
            read_length = recv( state->client_sockfd, ptr, max_length - *length, MSG_NOSIGNAL );
            if (read_length <= 0)
            {
                TcpClient_Disconnect( state );
                return TCPERR_CONNECTION_LOST;
            }

            ptr += read_length;
            *length += read_length;
        }
    }

    return TCPERR_NO_ERROR;
}

enum TcpErrorCode TcpClient_Write( struct TcpClientState *state, const void *data, uint32_t length )
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
            TcpClient_Disconnect( state );
            return TCPERR_CONNECTION_LOST;
        }

        ptr += write_length;
        remainder -= write_length;
    }
    if (remainder > 0)
    {
        return TCPERR_WRITING_TO_SERVER;
    }

    return TCPERR_NO_ERROR;
}
