//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#include "config.h"
#include "log.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _MSC_VER
#include <Windows.h>
#pragma warning(disable:4996)
#endif

#define MAX_LINE_LENGTH 128
#define MAX_NUM_ENTRIES 200
#define RECURSE_LIMIT 5

struct ConfigState
{
    uint32_t num_entries;
    char keys[MAX_NUM_ENTRIES][MAX_LINE_LENGTH];
    char values[MAX_NUM_ENTRIES][MAX_LINE_LENGTH];
    uint32_t recurse;
};

enum ErrorCode Config_Initialise( const char *filename, struct ConfigState **new_state )
{
    FILE *config_fp;
    char line_buffer[MAX_LINE_LENGTH];
    uint32_t line_number = 0;
    char key[MAX_LINE_LENGTH];
    char value[MAX_LINE_LENGTH];
    enum ErrorCode return_code = ERR_NO_ERROR;
    struct ConfigState *state = *new_state;

    if (state == NULL)
    {
        state = (struct ConfigState *)malloc( sizeof( *state ) );
        memset( state, 0, sizeof( *state ) );
    }

    config_fp = fopen( filename, "r" );
    if (config_fp == 0)
    {
        if (state->recurse == 0)
        {
            Log( ERROR, LCF, "Opening file '%s'", filename );
        }
        return ERR_FAILED_FILE_OPEN;
    }

    // Read each line until the end of the file
    while (fgets( line_buffer, MAX_LINE_LENGTH, config_fp ) != NULL)
    {
        line_number++;

        // Check we were able to read the entire line
        if (strlen( line_buffer ) == MAX_LINE_LENGTH - 1)
        {
            Log( ERROR, LCF, "Line %d too long in file '%s'", line_number, filename );
            return_code = ERR_CONFIG_LINE_TOO_LONG;
            break;
        }

        // Look for two strings separated by whitespace and an '=' character
        int items = sscanf( line_buffer, "%s = %s", key, value );
        if (items > 0)
        {
            // Find the key if it already exists, or go to the end of the list
            uint32_t entry = 0;
            while (entry < state->num_entries)
            {
                if (strncmp( key, state->keys[entry], MAX_LINE_LENGTH ) == 0)
                {
                    break;
                }
                ++entry;
            }

            // If the value is not set, remove it from the list
            if (items == 1 && entry < state->num_entries)
            {
                while (++entry < state->num_entries)
                {
                    strncpy( state->keys[entry - 1], state->keys[entry], MAX_LINE_LENGTH );
                    strncpy( state->values[entry - 1], state->values[entry], MAX_LINE_LENGTH );
                }
                state->num_entries--;
            }
            else if (items == 2)
            {
                // If it is a new entry, increment the number of entries
                if (entry == state->num_entries)
                {
                    if (state->num_entries++ == MAX_NUM_ENTRIES)
                    {
                        Log( ERROR, LCF, "Too many entries in file '%s'", filename );
                        return_code = ERR_CONFIG_FILE_TOO_LONG;
                        break;
                    }
                }
                else if (strcmp( state->values[entry], value ))
                {
                    Log( NOTE, LCF, "%s overrides '%s' from '%s' to '%s'",
                        filename, state->keys[entry], state->values[entry], value );
                }

                // Store the key and value
                strncpy( state->keys[entry], key, MAX_LINE_LENGTH );
                strncpy( state->values[entry], value, MAX_LINE_LENGTH );
            }
        }

        // Find include lines and recurse into them
        items = sscanf( line_buffer, "INCLUDE %[^\r\n]", value );
        if (items > 0)
        {
            if (state->recurse > RECURSE_LIMIT)
            {
                Log( ERROR, LCF, "Reached recursive include limit of %d", RECURSE_LIMIT );
                return_code = ERR_CONFIG_RECURSE_LIMIT;
                break;
            }

            state->recurse++;
            return_code = Config_Initialise( value, &state );
            state->recurse--;

            if (return_code && return_code != ERR_FAILED_FILE_OPEN)
            {
                // Return on any error other than failing to open the include file
                break;
            }
            return_code = ERR_NO_ERROR;
        }
    }

    fclose( config_fp );

    if (return_code == ERR_NO_ERROR)
    {
        *new_state = state;
    }

    return return_code;
}

enum ErrorCode Config_Get_String( struct ConfigState *state, char *key, char **value )
{
    if (state == NULL) return ERR_INVALID_STATE;

    // Compare the given key with each entry in the list of keys
    uint32_t entry = 0;
    while (entry < state->num_entries)
    {
        if (strncmp( key, state->keys[entry], MAX_LINE_LENGTH ) == 0)
        {
            break;
        }
        ++entry;
    }

    // If we went through the entire list without a match return an error
    if (entry == state->num_entries)
    {
        *value = NULL;
        Log( TRACE, LCF, "Looking for key '%s'", key );
        return ERR_CONFIG_KEY_NOT_FOUND;
    }

    *value = state->values[entry];

    return ERR_NO_ERROR;
}

enum ErrorCode Config_Get_Integer( struct ConfigState *state, char *key, int32_t *value )
{
    enum ErrorCode return_code;
    char *value_str;
    char *end_ptr;

    if (state == NULL) return ERR_INVALID_STATE;

    return_code = Config_Get_String( state, key, &value_str );
    if (return_code) return return_code;

    *value = (int32_t)strtol( value_str, &end_ptr, 0 );

    if (end_ptr == value_str)
    {
        Log( ERROR, LCF, "Key '%s' has value '%s'", key, value_str );
        return ERR_CONFIG_VALUE_NOT_INTEGER;
    }

    return ERR_NO_ERROR;
}

enum ErrorCode Config_Get_Float( struct ConfigState *state, char *key, float32_t *value )
{
    enum ErrorCode return_code;
    char *value_str;
    char *end_ptr;

    if (state == NULL) return ERR_INVALID_STATE;

    return_code = Config_Get_String( state, key, &value_str );
    if (return_code) return return_code;

    *value = strtof( value_str, &end_ptr );

    if (end_ptr == value_str)
    {
        Log( ERROR, LCF, "Key '%s' has value '%s'", key, value_str );
        return ERR_CONFIG_VALUE_NOT_FLOAT;
    }

    return ERR_NO_ERROR;
}
