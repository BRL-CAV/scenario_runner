//
//      Copyright (c) Fusion Processing Ltd 2014-2020
//      All rights reserved.
//

#pragma once

#include "float.h"

// Return the time in seconds since an unspecified start point.
float64_t Time_Get_Monotonic( void );

// Return the time in seconds since the Epoch. Will jump when clock is adjusted.
float64_t Time_Get_Realtime( void );

// Sleep for the given period. Causes a thread switch, so may sleep longer on a busy CPU.
void Time_Sleep( float32_t seconds );

// Measure statistics on the update rate and log the results
void Time_Update( void );
