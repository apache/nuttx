/****************************************************************************
 * include/stdint.h
 *
 *   Copyright (C) 2009, 2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_STDINT_H
#define __INCLUDE_STDINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_STDINT_H
#  include <arch/stdint.h>
#else
#  include <nuttx/compiler.h>
#  include <arch/types.h>
#  include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Limits of exact-width integer types */

#define INT8_MIN            (-INT8_MAX - 1)
#define INT8_MAX            127
#define UINT8_MAX           255

#define INT16_MIN           (-INT16_MAX - 1)
#define INT16_MAX           32767
#define UINT16_MAX          65535u

#ifdef __INT24_DEFINED
#  define INT24_MIN         (-INT24_MAX - 1)
#  define INT24_MAX         8388607
#  define UINT24_MAX        16777215u
#endif

#define INT32_MIN           (-INT32_MAX - 1)
#define INT32_MAX           2147483647
#define UINT32_MAX          4294967295u

#ifdef __INT64_DEFINED
#  define INT64_MIN         (-INT64_MAX - 1ll)
#  define INT64_MAX         9223372036854775807ll
#  define UINT64_MAX        18446744073709551615ull
#endif

/* Limits of minimum-width integer types */

#define INT_LEAST8_MIN      INT8_MIN
#define INT_LEAST8_MAX      INT8_MAX
#define UINT_LEAST8_MAX     UINT8_MAX

#define INT_LEAST16_MIN     INT16_MIN
#define INT_LEAST16_MAX     INT16_MAX
#define UINT_LEAST16_MAX    UINT16_MAX

#ifdef __INT24_DEFINED
#  define INT_LEAST24_MIN   INT24_MIN
#  define INT_LEAST24_MAX   INT24_MAX
#  define UINT_LEAST24_MAX  UINT24_MAX
#endif

#define INT_LEAST32_MIN     INT32_MIN
#define INT_LEAST32_MAX     INT32_MAX
#define UINT_LEAST32_MAX    UINT32_MAX

#ifdef __INT64_DEFINED
#  define INT_LEAST64_MIN   INT64_MIN
#  define INT_LEAST64_MAX   INT64_MAX
#  define UINT_LEAST64_MAX  UINT64_MAX
#endif

/* Limits of fastest minimum-width integer types */

#define INT_FAST8_MIN       INT8_MIN
#define INT_FAST8_MAX       INT8_MAX
#define UINT_FAST8_MAX      UINT8_MAX

#define INT_FAST16_MIN      INT16_MIN
#define INT_FAST16_MAX      INT16_MAX
#define UINT_FAST16_MAX     UINT16_MAX

#ifdef __INT24_DEFINED
#  define INT_FAST24_MIN    INT24_MIN
#  define INT_FAST24_MAX    INT24_MAX
#  define UINT_FAST24_MAX   UINT24_MAX
#endif

#define INT_FAST32_MIN      INT32_MIN
#define INT_FAST32_MAX      INT32_MAX
#define UINT_FAST32_MAX     UINT32_MAX

#ifdef __INT64_DEFINED
#  define INT_FAST64_MIN    INT64_MIN
#  define INT_FAST64_MAX    INT64_MAX
#  define UINT_FAST64_MAX   UINT64_MAX
#endif

/* Limits of integer types capable of holding object pointers */

#define INTPTR_MIN          PTR_MIN
#define INTPTR_MAX          PTR_MIN
#define UINTPTR_MAX         UPTR_MAX

/* Limits of greatest-width integer types */

#ifdef __INT64_DEFINED
#  define INTMAX_MIN        INT64_MIN
#  define INTMAX_MAX        INT64_MAX

#  define UINTMAX_MIN       UINT64_MIN
#  define UINTMAX_MAX       UINT64_MAX
#else
#  define INTMAX_MIN        INT32_MIN
#  define INTMAX_MAX        INT32_MAX

#  define UINTMAX_MIN       UINT32_MIN
#  define UINTMAX_MAX       UINT32_MAX
#endif

/* Macros for minimum-width integer constant expressions */

#if 0 /* REVISIT: Depends on architecture specific implementation */
#define INT8_C(x)           x
#define INT16_C(x)          x
#define INT32_C(x)          x ## l
#define INT64_C(x)          x ## ll

#define UINT8_C(x)          x
#define UINT16_C(x)         x
#define UINT32_C(x)         x ## ul
#define UINT64_C(x)         x ## ull
#endif

/* Macros for greatest-width integer constant expressions */

#ifdef CONFIG_HAVE_LONG_LONG
#  define INTMAX_C(x)       x ## ll
#  define UINTMAX_C(x)      x ## ull
#else
#  define INTMAX_C(x)       x ## l
#  define UINTMAX_C(x)      x ## ul
#endif

/* Limits of Other Integer Types */

#if 0
#  define                   PTRDIFF_MIN
#  define                   PTRDIFF_MAX
#endif

#if 0
#  define                   WCHAR_MIN
#  define                   WCHAR_MAX

#  define                   WINT_MIN
#  define                   WINT_MAX
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Exact-width integer types.  NOTE that these types are defined in
 * architecture-specific logic with leading underscore character. This file
 * typedef's these to the final name without the underscore character.  This
 * roundabout way of doings things allows the stdint.h to be removed from the
 * include/ directory in the event that the user prefers to use the definitions
 * provided by their toolchain header files.
 */

typedef _int8_t             int8_t;
typedef _uint8_t            uint8_t;

typedef _int16_t            int16_t;
typedef _uint16_t           uint16_t;

#ifdef __INT24_DEFINED
typedef _int24_t            int24_t;
typedef _uint24_t           uint24_t;
#endif

typedef _int32_t            int32_t;
typedef _uint32_t           uint32_t;

#ifdef __INT64_DEFINED
typedef _int64_t            int64_t;
typedef _uint64_t           uint64_t;
#endif

/* Minimum-width integer types */

typedef _int8_t             int_least8_t;
typedef _uint8_t            uint_least8_t;

typedef _int16_t            int_least16_t;
typedef _uint16_t           uint_least16_t;

#ifdef __INT24_DEFINED
typedef _int24_t            int_least24_t;
typedef _uint24_t           uint_least24_t;
#else
typedef _int32_t            int_least24_t;
typedef _uint32_t           uint_least24_t;
#endif

typedef _int32_t            int_least32_t;
typedef _uint32_t           uint_least32_t;

#ifdef __INT64_DEFINED
typedef _int64_t            int_least64_t;
typedef _uint64_t           uint_least64_t;
#endif

/* Fastest minimum-width integer types */

typedef _int8_t             int_fast8_t;
typedef _uint8_t            uint_fast8_t;

typedef int                 int_fast16_t;
typedef unsigned int        uint_fast16_t;

#ifdef __INT24_DEFINED
typedef _int24_t            int_fast24_t;
typedef _uint24_t           uint_fast24_t;
#else
typedef _int32_t            int_fast24_t;
typedef _uint32_t           uint_fast24_t;
#endif

typedef _int32_t            int_fast32_t;
typedef _uint32_t           uint_fast32_t;

#ifdef __INT64_DEFINED
typedef _int64_t            int_fast64_t;
typedef _uint64_t           uint_fast64_t;
#endif

/* Integer types capable of holding object pointers */

typedef _intptr_t           intptr_t;
typedef _uintptr_t          uintptr_t;

/* Some architectures support a FAR pointer which is larger then the normal
 * (near) pointer
 */

#ifdef CONFIG_HAVE_FARPOINTER
typedef _int_farptr_t       int_farptr_t;
typedef _uint_farptr_t      uint_farptr_t;
#endif

/* Greatest-width integer types */

#ifdef __INT64_DEFINED
typedef _int64_t            intmax_t;
typedef _uint64_t           uintmax_t;
#else
typedef _int32_t            intmax_t;
typedef _uint32_t           uintmax_t;
#endif

#endif /* CONFIG_ARCH_STDBOOL_H */
#endif /* __INCLUDE_STDINT_H */
