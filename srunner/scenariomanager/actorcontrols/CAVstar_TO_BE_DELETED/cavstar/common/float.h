//
//      Copyright (c) Fusion Processing Ltd 2020
//      All rights reserved.
//

#pragma once

#if __SIZEOF_FLOAT__ == 4
typedef float float32_t;
#elif _M_IX86 == 600 || _M_AMD64 == 100
typedef float float32_t;
#else
#error Failed to define float32_t
#endif

#if __SIZEOF_DOUBLE__ == 8
typedef double float64_t;
#elif _M_IX86 == 600 || _M_AMD64 == 100
typedef double float64_t;
#else
#error Failed to define float64_t
#endif
