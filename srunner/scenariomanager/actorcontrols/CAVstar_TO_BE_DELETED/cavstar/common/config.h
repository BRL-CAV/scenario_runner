//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "float.h"
#include "error_codes.h"

#include <stdint.h>

#ifndef DLL_PUBLIC
#define DLL_PUBLIC
#endif // DLL_PUBLIC

struct ConfigState;

// Allocates a state object if it is NULL and initialises the config
// Returns 0 on success, otherwise an error code
DLL_PUBLIC enum ErrorCode Config_Initialise( const char *filename, struct ConfigState **state );

// Searches for the given key in the config file loaded during initialisation
// Provides a pointer to the string value associated with the key
// Returns 0 on success, otherwise an error code
DLL_PUBLIC enum ErrorCode Config_Get_String( struct ConfigState *state, char *key, char **value );

// Searches for the given key in the config file loaded during initialisation
// Provides a pointer to the integer value associated with the key
// Returns 0 on success, otherwise an error code
DLL_PUBLIC enum ErrorCode Config_Get_Integer( struct ConfigState *state, char *key, int32_t *value );

// Searches for the given key in the config file loaded during initialisation
// Provides a pointer to the floating point value associated with the key
// Returns 0 on success, otherwise an error code
DLL_PUBLIC enum ErrorCode Config_Get_Float( struct ConfigState *state, char *key, float32_t *value );
