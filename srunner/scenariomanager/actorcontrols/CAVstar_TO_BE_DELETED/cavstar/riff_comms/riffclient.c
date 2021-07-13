//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#include "riffclient.h"
#include "tcpclient.h"
#include "common/float.h"
#include "common/error_codes.h"
#include "common/log.h"
#include "common/time.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

struct Chunk
{
    char header[TAG_LENGTH + sizeof( uint32_t )];
    void *data;
    uint32_t data_length;
};

struct RiffClientState
{
    float64_t timeout;
    float64_t last_waiting_time;
    float64_t last_receive_time;
    float64_t last_ping_time;
    struct TcpClientState *tcpclient;

    void *data;
    char *name;
    uint32_t data_length;

    uint32_t padding;
};

enum ErrorCode RiffClient_Initialise( int32_t port, char *host, float64_t timeout, char *name, struct RiffClientState **new_state )
{
    enum TcpErrorCode tcp_error_code;
    struct RiffClientState *state = *new_state;

    if (state == NULL)
    {
        state = (struct RiffClientState *)malloc( sizeof( *state ) );
        if (state == NULL) return ERR_FAILED_MALLOC;
        memset( state, 0, sizeof( *state ) );
        *new_state = state;
    }

    if (name == NULL || strlen( name ) <= 0 || strlen( name ) > 80)
    {
        Log( ERROR, LRC, "Invalid name to RiffClient_Initialise (%s)", name );
        return ERR_INVALID_PARAMETER;
    }

    Log( TRACE, LRC, "Initialising client for %s...", name );

    state->last_waiting_time = Time_Get_Monotonic();

    tcp_error_code = TcpClient_Initialise( host, (uint16_t)port, &state->tcpclient );
    if (tcp_error_code) return ERR_SOCKET_ERROR;

    state->timeout = timeout;

    if (state->name) free( state->name );
    state->name = malloc( strlen( name ) + 1 );
    if (state->name) memcpy( state->name, name, strlen( name ) + 1 );

    return ERR_NO_ERROR;
}

enum ErrorCode RiffClient_Connect( struct RiffClientState *state )
{
    enum TcpErrorCode tcp_error_code;
    _Bool connected;
    float64_t current_time;

    if (state == NULL) return ERR_INVALID_STATE;

    tcp_error_code = TcpClient_Get_Connected( state->tcpclient, &connected );

    if (!tcp_error_code && !connected)
    {
        tcp_error_code = TcpClient_Connect( state->tcpclient, state->timeout, &connected );

        current_time = Time_Get_Monotonic();
        if (!tcp_error_code && connected)
        {
            Log( NOTE, LRC, "Connected to %s server", state->name );
            state->last_ping_time = current_time;
            state->last_receive_time = current_time;
        }
        else if (current_time > state->last_waiting_time + 1)
        {
            Log( TRACE, LRC, "Waiting for %s server", state->name );
            state->last_waiting_time = current_time;
        }
    }
    if (tcp_error_code) return ERR_SOCKET_ERROR;

    return ERR_NO_ERROR;
}

enum ErrorCode RiffClient_Disconnect( struct RiffClientState *state )
{
    enum TcpErrorCode tcp_error_code;

    if (state == NULL) return ERR_INVALID_STATE;

    Log( TRACE, LRC, "Disconnecting from %s server...", state->name );

    tcp_error_code = TcpClient_Disconnect( state->tcpclient );
    if (tcp_error_code) return ERR_SOCKET_ERROR;

    return ERR_NO_ERROR;
}

void RiffClient_Stop( struct RiffClientState *state )
{
    if (state == NULL) return;

    TcpClient_Disconnect( state->tcpclient );
    free( state->tcpclient );
    state->tcpclient = NULL;

    free( state->name );
    state->name = NULL;
}

enum ErrorCode RiffClient_Get_Connected( struct RiffClientState *state, _Bool *connected )
{
    enum TcpErrorCode tcp_error_code;

    if (state == NULL) return ERR_INVALID_STATE;

    tcp_error_code = TcpClient_Get_Connected( state->tcpclient, connected );
    if (tcp_error_code) return ERR_SOCKET_ERROR;

    return ERR_NO_ERROR;
}

enum ErrorCode RiffClient_Receive( struct RiffClientState *state, char tag[TAG_LENGTH + 1], void **data, uint32_t *data_length )
{
    enum ErrorCode error_code = ERR_NO_ERROR;
    enum TcpErrorCode tcp_error_code;
    _Bool connected;
    uint32_t length = 0;
    float64_t current_time = Time_Get_Monotonic();

    if (state == NULL) return ERR_INVALID_STATE;

    tcp_error_code = TcpClient_Get_Connected( state->tcpclient, &connected );

    if (!tcp_error_code && connected)
    {
        // Check if a message has been received via ethernet (non-blocking)
        tcp_error_code = TcpClient_Read( state->tcpclient, 0, tag, TAG_LENGTH, &length );
        if (tcp_error_code == TCPERR_CONNECTION_LOST)
        {
            Log( WARNING, LRC, "Connection to %s server lost", state->name );
            error_code = ERR_CONNECTION_LOST; // Calling function calls ErrorHandler
        }
        else if (current_time > state->last_receive_time + state->timeout
            && current_time > state->last_ping_time + state->timeout / 2)
        {
            Log( WARNING, LRC, "Connection to %s server timed out (%.2f, %.2f)", state->name,
                current_time - state->last_receive_time, current_time - state->last_ping_time );
            TcpClient_Disconnect( state->tcpclient );
            error_code = ERR_CONNECTION_LOST; // Calling function calls ErrorHandler
        }
    }

    tag[TAG_LENGTH] = 0;
    tag[length] = 0;
    *data = NULL;
    *data_length = 0;

    if (!tcp_error_code && length > 0)
    {
        // Check the tag is valid
        for (uint32_t i = 0; i < TAG_LENGTH; ++i)
        {
            if (tag[i] < ' ' || tag[i] > 'z')
            {
                // Flush the read buffer to try to realign with the messages
                uint8_t black_hole[1024];
                while (length > 0)
                {
                    TcpClient_Read( state->tcpclient, 0, black_hole, 1024, &length );
                }
                Log( ERROR, LRC, "Invalid tag received '%s'", tag );
                error_code = ERR_INVALID_TAG;
            }
        }

        // Read the length of data associated with the message
        if (error_code == ERR_NO_ERROR)
        {
            tcp_error_code = TcpClient_Read( state->tcpclient, state->timeout, data_length, sizeof( *data_length ), &length );
            if (tcp_error_code) error_code = ERR_SOCKET_ERROR;

            if (!tcp_error_code)
            {
                if (length != sizeof( *data_length ))
                {
                    Log( ERROR, LRC, "Error reading data length" );
                    error_code = ERR_INVALID_DATA;
                }
                // A data length over 16 MiB is assumed to be invalid
                else if (*data_length > 0x01000000)
                {
                    Log( ERROR, LRC, "Error invalid data length %d", *data_length );
                    error_code = ERR_INVALID_DATA;
                }
            }
        }

        if (!error_code && *data_length > state->data_length)
        {
            state->data = realloc( state->data, *data_length );
            if (state->data == NULL) error_code = ERR_FAILED_MALLOC;
            state->data_length = *data_length;
        }

        // Read the data associated with the command
        if (!error_code && *data_length > 0)
        {
            tcp_error_code = TcpClient_Read( state->tcpclient, state->timeout, state->data, *data_length, &length );
            if (tcp_error_code) error_code = ERR_SOCKET_ERROR;

            if (!tcp_error_code)
            {
                if (length != *data_length)
                {
                    Log( ERROR, LRC, "Error reading data" );
                    error_code = ERR_INVALID_DATA;
                }
                else
                {
                    *data = state->data;
                }
            }
        }
        else if (strncmp( tag, "PING", TAG_LENGTH ) == 0)
        {
            // Reply to PING to confirm the connection
            tag[0] = 0;

            error_code = RiffClient_Write( state, "PONG", NULL, 0 );
        }
        else if (strncmp( tag, "PONG", TAG_LENGTH ) == 0)
        {
            // Do not return PONG messages - they are used to test the connection
            tag[0] = 0;
        }

        if (error_code == ERR_NO_ERROR)
        {
            state->last_receive_time = current_time;
        }
    }
    else if (current_time > state->last_receive_time + state->timeout / 2
        && current_time > state->last_ping_time + state->timeout / 2 && connected)
    {
        // If we have not received for a while, send a PING to check the connection
        state->last_ping_time = current_time;
        error_code = RiffClient_Write( state, "PING", NULL, 0 );
    }

    return error_code;
}

enum ErrorCode RiffClient_Write( struct RiffClientState *state, char tag[TAG_LENGTH], const void *data, uint32_t data_length )
{
    enum ErrorCode error_code = ERR_NO_ERROR;
    enum TcpErrorCode tcp_error_code;
    char header[TAG_LENGTH + sizeof( data_length )];
    _Bool connected;

    if (state == NULL) return ERR_INVALID_STATE;

    // Create a header with the tag and data length
    memcpy( &header[0], tag, TAG_LENGTH );
    memcpy( &header[TAG_LENGTH], &data_length, sizeof( data_length ) );

    tcp_error_code = TcpClient_Get_Connected( state->tcpclient, &connected );

    //Log( DEBUG, LRC, "Sending tag %s with %d bytes to %s", tag, data_length, state->name );

    // Write header followed by data
    if (!tcp_error_code && connected)
    {
        tcp_error_code = TcpClient_Write( state->tcpclient, header, sizeof( header ) );
        if (tcp_error_code == TCPERR_CONNECTION_LOST)
        {
            Log( WARNING, LRC, "Connection to %s server lost", state->name );
            error_code = ERR_CONNECTION_LOST; // Calling function calls ErrorHandler
        }
        if (!tcp_error_code)
        {
            tcp_error_code = TcpClient_Write( state->tcpclient, data, data_length );
        }
    }
    if (tcp_error_code) error_code = ERR_SOCKET_ERROR;

    return error_code;
}
