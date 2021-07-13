//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "common/float.h"
#include "common/error_codes.h"

#include <stdint.h>

#ifndef DLL_PUBLIC
#define DLL_PUBLIC
#endif // DLL_PUBLIC

#define TAG_LENGTH 4

struct RiffServerState;

// Allocates a state object if it is NULL and initialises the RIFF server
DLL_PUBLIC enum ErrorCode RiffServer_Initialise( int32_t port, float64_t timeout, char *name, struct RiffServerState **state );

// Waits for a connection on the initialised socket
DLL_PUBLIC enum ErrorCode RiffServer_Listen( struct RiffServerState *state );

// Clones the state without the connected client, ready to listen again
DLL_PUBLIC enum ErrorCode RiffServer_Fork( struct RiffServerState *state, struct RiffServerState **new_state );

// Disconnects the current client ready to listen again
DLL_PUBLIC enum ErrorCode RiffServer_Disconnect( struct RiffServerState *state );

// Closes the connection
DLL_PUBLIC void RiffServer_Stop( struct RiffServerState *state );

// Returns the internal 'connected' state, which is updated on Receive or Write
DLL_PUBLIC enum ErrorCode RiffServer_Get_Connected( struct RiffServerState *state, _Bool *connected );

// Checks for a message from the RIFF client and if there is one,
// sets the pointer to point to any data associated with the message
DLL_PUBLIC enum ErrorCode RiffServer_Receive( struct RiffServerState *state, char tag[TAG_LENGTH + 1], void **data, uint32_t *data_length );

// Writes a tag followed by an array of bytes to the RIFF client
DLL_PUBLIC enum ErrorCode RiffServer_Write( struct RiffServerState *state, char tag[TAG_LENGTH], const void *data, uint32_t data_length );
