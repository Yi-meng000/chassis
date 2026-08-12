

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
#include "stm32h7xx_hal.h"

#define __RAM_D1_ __attribute__((section(".RAM_D1")))
#define __RAM_D2_ __attribute__((section(".RAM_D2")))
#define __RAM_D3_ __attribute__((section(".RAM_D3")))

#define ALIGN_32B __attribute__((aligned(32)))

    typedef int32_t s32;
    typedef int16_t s16;
    typedef int8_t s8;

    typedef const int32_t sc32;
    typedef const int16_t sc16;
    typedef const int8_t sc8;

    typedef __IO int32_t vs32;
    typedef __IO int16_t vs16;
    typedef __IO int8_t vs8;

    typedef __I int32_t vsc32;
    typedef __I int16_t vsc16;
    typedef __I int8_t vsc8;

    typedef uint32_t u32;
    typedef uint16_t u16;
    typedef uint8_t u8;

    typedef const uint32_t uc32;
    typedef const uint16_t uc16;
    typedef const uint8_t uc8;

    typedef __IO uint32_t vu32;
    typedef __IO uint16_t vu16;
    typedef __IO uint8_t vu8;

    typedef __I uint32_t vuc32;
    typedef __I uint16_t vuc16;
    typedef __I uint8_t vuc8;

#define USE_ZMDR 0
#define USE_VESC 0
#define USE_DJ 0
#define USE_UNITREE 0

#ifdef __cplusplus
}
#endif

#endif
