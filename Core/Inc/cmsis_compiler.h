#ifndef CMSIS_COMPILER_H
#define CMSIS_COMPILER_H

#include "gd32f4xx.h"

#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE static inline __attribute__((always_inline))
#endif

#ifndef __USED
#define __USED __attribute__((used))
#endif

#ifndef __PACKED
#define __PACKED __attribute__((packed))
#endif

#ifndef __ALIGNED
#define __ALIGNED(x) __attribute__((aligned(x)))
#endif

#endif /* CMSIS_COMPILER_H */