//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "tcperror.h"
#include "common/float.h"

#include <stdint.h>

struct TcpClientState;

// Allocates a state object if it is NULL and initialises the client
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Initialise( char *hostname, uint16_t port, struct TcpClientState **state );

// Attempts to connect to a server and sets 'connected' if successful
// Returns immediately. Subsequent calls continue same connection until
// timeout milliseconds have elapsed, when a new connection is started
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Connect( struct TcpClientState *state, float64_t timeout, _Bool *connected );

// Disconnects from the server
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Disconnect( struct TcpClientState *state );

// Returns the internal 'connected' state, which is updated on Receive or Write
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Get_Connected( struct TcpClientState *state, _Bool *connected );

// Waits for data and then reads up to max_length bytes to data address
// from the server. Actual number of bytes read provided in *length
// The read will wait for the specified number of seconds before
// returning with no error and length set to zero if no data is received.
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Read( struct TcpClientState *state, float64_t timeout, void *data, uint32_t max_length, uint32_t *length );

// Writes the specified number of bytes from the data address to the server
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpClient_Write( struct TcpClientState *state, const void *data, uint32_t length );
