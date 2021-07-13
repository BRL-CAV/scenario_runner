//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "tcperror.h"
#include "common/float.h"

#include <stdint.h>

struct TcpServerState;

// Allocates a state object if it is NULL and opens a listening port
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Initialise( uint16_t port, struct TcpServerState **state );

// Waits for a client to connect
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_ClientConnect( struct TcpServerState *state, _Bool *connected );

// Clones the state without the connected client, ready to listen again
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Fork( struct TcpServerState *state, struct TcpServerState **new_state );

// Disconnects a client
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Disconnect( struct TcpServerState *state );

// Disconnects a client, closes the listening socket and releases the state object
void TcpServer_Close( struct TcpServerState *state );

// Returns the internal 'connected' state, which is updated on Receive or Write
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Get_Connected( struct TcpServerState *state, _Bool *connected );

// Waits for data and then reads up to max_length bytes to data address
// from the client. Actual number of bytes read provided in *length
// The read will wait for the specified number of seconds before
// returning with no error and length set to zero if no data is received.
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Read( struct TcpServerState *state, float64_t timeout, void *data, uint32_t max_length, uint32_t *length );

// Writes the specified number of bytes from the data address to the client
// Returns 0 on success, otherwise an error code
enum TcpErrorCode TcpServer_Write( struct TcpServerState *state, const void *data, uint32_t length );
