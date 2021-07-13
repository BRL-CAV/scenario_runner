//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#include "time.h"
//#include "error.h"
#include "log.h"

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#if defined GTEST
static float64_t global_current_time = 0;
#elif defined _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

float64_t Time_Get_Monotonic( void )
{
    float64_t now;
#if defined GTEST
    now = global_current_time;
#elif defined _WIN32
    static LARGE_INTEGER count, frequency = { 0 };
    if (frequency.QuadPart == 0) QueryPerformanceFrequency( &frequency );
    QueryPerformanceCounter( &count );
    now = count.QuadPart / (float64_t)frequency.QuadPart;
#else
    struct timespec monotonic_timespec;
    clock_gettime( CLOCK_MONOTONIC, &monotonic_timespec );
    now = monotonic_timespec.tv_sec + monotonic_timespec.tv_nsec / (1e9);
#endif
    return now;
}

float64_t Time_Get_Realtime( void )
{
    float64_t now;
#if defined GTEST
    now = global_current_time;
#elif defined _WIN32
    //TODO find a method to get the wall time with fractional seconds then use in logging
    static LARGE_INTEGER count, frequency = { 0 };
    if (frequency.QuadPart == 0) QueryPerformanceFrequency( &frequency );
    QueryPerformanceCounter( &count );
    now = count.QuadPart / (float64_t)frequency.QuadPart;
#else
    struct timespec realtime_timespec;
    clock_gettime( CLOCK_REALTIME, &realtime_timespec );
    now = realtime_timespec.tv_sec + realtime_timespec.tv_nsec / (1e9);
#endif
    return now;
}

void Time_Sleep( float32_t seconds )
{
#if defined GTEST
    global_current_time += (float64_t)seconds;
#elif defined _WIN32
    Sleep( (DWORD)(seconds * 1e3f) );
#else
    usleep( (useconds_t)(seconds * 1e6f) );
#endif
}

void Time_Update( void )
{
    static float64_t last_time = 0;
    float64_t current_time = Time_Get_Monotonic();
    float64_t interval = current_time - last_time;
    if (last_time < 0.1) interval = 0;
    last_time = current_time;
    if (interval > 0.1)
    {
        Log( WARNING, LEH, "Slow iteration: %.3f seconds", interval );
        //ErrorHandler( ERR_SLOW_ITERATION, __LINE__, __FILE__ );
    }
}
