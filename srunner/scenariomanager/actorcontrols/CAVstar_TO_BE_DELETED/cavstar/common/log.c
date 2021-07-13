//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

//TODO Move library calls out of critical code

#include "log.h"
#include "log_message.h"
#include "error_codes.h"
#include "config.h"
#include "riff_comms/riffserver.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif

// Log level to use if not otherwise set
#define DEFAULT_LOG_LEVEL NOTE

static struct LogState
{
    struct RiffServerState *riffserver;
    FILE *logfile;
    enum LogLevel log_level[NUM_LOG_SOURCES];
    enum LogLevel padding[32 - NUM_LOG_SOURCES];
} global_log_state = { 0, 0, { 0 }, { 0 } };

enum ErrorCode Log_Initialise( struct ConfigState *config )
{
    enum ErrorCode error_code;

    for (uint32_t src = 0; src < NUM_LOG_SOURCES; src++)
    {
        char *log_level_str = "Unknown";
        switch (src)
        {
        case LEH: log_level_str = "log_level_error_handler"; break;
        case LCF: log_level_str = "log_level_config"; break;
        case LRC: log_level_str = "log_level_riff_comms"; break;
        case LIC: log_level_str = "log_level_intercept"; break;
        case LMC: log_level_str = "log_level_main_critical"; break;
        case LMN: log_level_str = "log_level_main_non_critical"; break;
        case LSS: log_level_str = "log_level_simple_sim"; break;
        case LFI: log_level_str = "log_level_fms_interface"; break;
        case LSP: log_level_str = "log_level_strategic_planner"; break;
        case LTP: log_level_str = "log_level_tactical_planner"; break;
        case LVC: log_level_str = "log_level_vehicle_controller"; break;
        case LVI: log_level_str = "log_level_vehicle_interface"; break;
        case LLC: log_level_str = "log_level_localizer"; break;
        case LMD: log_level_str = "log_level_map_data"; break;
        case LGS: log_level_str = "log_level_gnss_sensor"; break;
        case LMS: log_level_str = "log_level_mems_sensor"; break;
        case LTL: log_level_str = "log_level_traffic_light"; break;
        case LLD: log_level_str = "log_level_lane_detector"; break;
        case LOD: log_level_str = "log_level_object_detector"; break;
        case NUM_LOG_SOURCES: log_level_str = "invalid"; break;
            // No default: - all enumerations must be handled explicitly
        }

        int32_t log_level_int;
        error_code = Config_Get_Integer( config, log_level_str, &log_level_int );
        if (error_code == ERR_CONFIG_KEY_NOT_FOUND) continue;
        if (error_code) return error_code;

        global_log_state.log_level[src] = (enum LogLevel)log_level_int;
    }

    return ERR_NO_ERROR;
}

void Log_Set_RiffServer( struct RiffServerState *state )
{
    global_log_state.riffserver = state;
}

void Log_Set_Logfile( FILE *file )
{
    global_log_state.logfile = file;
}

void Log( enum LogLevel log_level, enum LogSource log_source, const char *format, ... )
{
    size_t local_format_length;
    char *local_format;
    va_list argptr;

    //TODO find a method to get the wall time with fractional seconds for Time_Get_Realtime and use here
    time_t raw_time = time( NULL );
    struct tm *tm_time;
    char time_str[] = "00:00:00";
#ifdef _MSC_VER
    struct tm local_tm_time;
    tm_time = &local_tm_time;
    gmtime_s( tm_time, &raw_time );
#else
    tm_time = gmtime( &raw_time );
#endif
    if (strftime( time_str, sizeof( time_str ), "%H:%M:%S", tm_time ) == 0)
    {
        *time_str = '\0';
    }

    char *source_str = "Unknown";
    switch (log_source)
    {
    case LEH: source_str = " E-handler"; break;
    case LCF: source_str = " Config"; break;
    case LRC: source_str = " RIFF-comm"; break;
    case LIC: source_str = " Intercept"; break;
    case LMC: source_str = " M-crit"; break;
    case LMN: source_str = " M-non-crit"; break;
    case LSS: source_str = " Simple-sim"; break;
    case LFI: source_str = " FMS-iface"; break;
    case LSP: source_str = " Strat-plan"; break;
    case LTP: source_str = " Tact-plan"; break;
    case LVC: source_str = " V-control"; break;
    case LVI: source_str = " V-iface"; break;
    case LLC: source_str = " Localizer"; break;
    case LMD: source_str = " Map-data"; break;
    case LGS: source_str = " GNSS"; break;
    case LMS: source_str = " MEMS"; break;
    case LTL: source_str = " T-light"; break;
    case LLD: source_str = " Lane-dect"; break;
    case LOD: source_str = " Obj-dect"; break;
    case NUM_LOG_SOURCES: source_str = " invalid"; break;
        // No default: - all enumerations must be handled explicitly
    }

    local_format_length = strlen( time_str ) + strlen( source_str ) + strlen( " Warning: " ) + strlen( format );
    local_format = (char *)malloc( local_format_length + 2 );
    if (local_format == NULL)
    {
        printf( "%s %s Error: Failed Malloc\n", time_str, source_str );
        return;
    }
    memset( local_format, 0, local_format_length + 2 );

    switch (log_level)
    {
    case LL_NOT_SET: snprintf( local_format, local_format_length, "%s%s: ", time_str, source_str ); break;
    case ERROR:   snprintf( local_format, local_format_length, "%s%s Error: ", time_str, source_str ); break;
    case WARNING: snprintf( local_format, local_format_length, "%s%s Warning: ", time_str, source_str ); break;
    case NOTE:    snprintf( local_format, local_format_length, "%s%s Note: ", time_str, source_str ); break;
    case TRACE:   snprintf( local_format, local_format_length, "%s%s Trace: ", time_str, source_str ); break;
    case DEBUG:   snprintf( local_format, local_format_length, "%s%s Debug: ", time_str, source_str ); break;
        // No default: - all enumerations must be handled explicitly
    }

#ifdef _MSC_VER
    strncat_s( local_format, local_format_length + 2, format, local_format_length - strlen( local_format ) );
    strncat_s( local_format, local_format_length + 2, "\n", local_format_length - strlen( local_format ) + 2 );
#else
    strncat( local_format, format, local_format_length - strlen( local_format ) );
    strncat( local_format, "\n", local_format_length - strlen( local_format ) + 2 );
#endif

    if ((global_log_state.log_level[log_source] == LL_NOT_SET && log_level <= DEFAULT_LOG_LEVEL) ||
        (global_log_state.log_level[log_source] != LL_NOT_SET && log_level <= global_log_state.log_level[log_source]))
    {
        va_start( argptr, format );
        vprintf( local_format, argptr );
        va_end( argptr );

        if (global_log_state.logfile)
        {
            va_start( argptr, format );
            vfprintf( global_log_state.logfile, local_format, argptr );
            va_end( argptr );
            fflush( global_log_state.logfile );
        }

        if (global_log_state.riffserver)
        {
            struct LogMessage log_message;
            log_message.log_level = log_level;
            log_message.log_source = log_source;

            va_start( argptr, format );
            vsnprintf( log_message.log_text, MAX_LOG_MESSAGE_LENGTH, format, argptr );
            va_end( argptr );

            RiffServer_Write( global_log_state.riffserver, LOG_MESSAGE_TAG, &log_message, sizeof( log_message ) );
        }
    }

    free( local_format );
}
