//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "error_codes.h"

#include <stdio.h>

struct ConfigState;
struct RiffServerState;

enum LogLevel
{
    LL_NOT_SET = 0, // Must be 0 to initialise array
    ERROR = 1,
    WARNING = 2,
    NOTE = 3,
    TRACE = 4,
    DEBUG = 5
};

enum LogSource
{
    LEH = 0, // Error handler
    LCF, // Config
    LRC, // RIFF comms
    LIC, // Intercept
    LMC, // Main critical
    LMN, // Main non-critical
    LSS, // Simple simulator
    LFI, // FMS Interface
    LSP, // Strategic planner
    LTP, // Tactical planner
    LVC, // Vehicle controller
    LVI, // Vehicle interface
    LLC, // Localizer
    LMD, // Map data
    LGS, // GNSS sensor
    LMS, // MEMS sensor
    LTL, // Traffic light
    LLD, // Lane detector
    LOD, // Object detector
    NUM_LOG_SOURCES // Used to size the array of log levels
};

// Configure the logging system. Logging will use default values until configured
enum ErrorCode Log_Initialise( struct ConfigState *config );

// Set a RIFF server to send log messages to
void Log_Set_RiffServer( struct RiffServerState *state );

// Set file to write log messages to
void Log_Set_Logfile( FILE *file );

// The formatted string is prepended with a timestamp, log level, log source,
// terminated with a new line and logged via the logging system
void Log( enum LogLevel log_level, enum LogSource log_source, const char *format, ... );
