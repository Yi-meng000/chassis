

#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mathFunc.h"

#define __RAM_D1_ __attribute__((section(".RAM_D1")))
#define __RAM_D2_ __attribute__((section(".RAM_D2")))
#define __RAM_D3_ __attribute__((section(".RAM_D3")))

#define ALIGN_32B __attribute__((aligned(32)))
#define USE_DJ 0
#define USE_VESC 0
#define USE_ZMDR 1

#ifdef __cplusplus
}
#endif

#endif
