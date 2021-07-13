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

struct RiffClientState;

// Allocates a state object if it is NULL and initialises the RIFF client
DLL_PUBLIC enum ErrorCode RiffClient_Initialise( int32_t port, char *host, float64_t timeout, char *name, struct RiffClientState **state );

// Connects to the server using the initialised socket
DLL_PUBLIC enum ErrorCode RiffClient_Connect( struct RiffClientState *state );

// Disconnects from the server
DLL_PUBLIC enum ErrorCode RiffClient_Disconnect( struct RiffClientState *state );

// Closes the connection
DLL_PUBLIC void RiffClient_Stop( struct RiffClientState *state );

// Returns the internal 'connected' state, which is updated on Receive or Write
DLL_PUBLIC enum ErrorCode RiffClient_Get_Connected( struct RiffClientState *state, _Bool *connected );

// Checks for a message from the RIFF server and if there is one,
// sets the pointer to point to any data associated with the message
DLL_PUBLIC enum ErrorCode RiffClient_Receive( struct RiffClientState *state, char tag[TAG_LENGTH + 1], void **data, uint32_t *data_length );

// Writes a tag followed by an array of bytes to the RIFF server
DLL_PUBLIC enum ErrorCode RiffClient_Write( struct RiffClientState *state, char tag[TAG_LENGTH], const void *data, uint32_t data_length );
