/****************************************************************************
 * include/stdint.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#  include <arch/inttypes.h>
#  include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Limits of exact-width integer types */

#define INT8_MIN            (-INT8_MAX - 1)
#define INT8_MAX            INT8_C(127)
#define UINT8_MAX           UINT8_C(255)

#define INT16_MIN           (-INT16_MAX - 1)
#define INT16_MAX           INT16_C(32767)
#define UINT16_MAX          UINT16_C(65535)

#ifdef __INT24_DEFINED
#  define INT24_MIN         (-INT24_MAX - 1)
#  define INT24_MAX         INT24_C(8388607)
#  define UINT24_MAX        UINT24_C(16777215)
#endif

#define INT32_MIN           (-INT32_MAX - 1)
#define INT32_MAX           INT32_C(2147483647)
#define UINT32_MAX          UINT32_C(4294967295)

#ifdef __INT64_DEFINED
#  define INT64_MIN         (-INT64_MAX - 1)
#  define INT64_MAX         INT64_C(9223372036854775807)
#  define UINT64_MAX        UINT64_C(18446744073709551615)
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
#define INTPTR_MAX          PTR_MAX
#define UINTPTR_MAX         UPTR_MAX

/* Limits of greatest-width integer types */

#ifdef __INT64_DEFINED
#  define INTMAX_MIN        INT64_MIN
#  define INTMAX_MAX        INT64_MAX

#  define UINTMAX_MIN       UINT64_MIN
#  define UINTMAX_MAX       UINT64_MAX

#  define INTMAX_C(x)       INT64_C(x)
#  define UINTMAX_C(x)      UINT64_C(x)
#else
#  define INTMAX_MIN        INT32_MIN
#  define INTMAX_MAX        INT32_MAX

#  define UINTMAX_MIN       UINT32_MIN
#  define UINTMAX_MAX       UINT32_MAX

#  define INTMAX_C(x)       INT32_C(x)
#  define UINTMAX_C(x)      UINT32_C(x)
#endif

#ifdef __INT64_DEFINED
#  define PTRDIFF_MIN       INT64_MIN
#  define PTRDIFF_MAX       INT64_MAX
#else
#  define PTRDIFF_MIN       INT32_MIN
#  define PTRDIFF_MAX       INT32_MAX
# endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Exact-width integer types.  NOTE that these types are defined in
 * architecture-specific logic with leading underscore character. This file
 * typedef's these to the final name without the underscore character.  This
 * roundabout way of doings things allows the stdint.h to be removed from the
 * include/ directory in the event that the user prefers to use the
 * definitions provided by their toolchain header files.
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

/* Integer types capable of holding object pointers
 * As a general rule, the size of size_t should be the same as the size of
 * uintptr_t: 32-bits on a machine with 32-bit addressing but 64-bits on a
 * machine with 64-bit addressing.
 */

typedef _ssize_t            intptr_t;
typedef _size_t             uintptr_t;

/* Some architectures support a FAR pointer which is larger then the normal
 * (near) pointer
 */

#ifdef CONFIG_HAVE_FARPOINTER
typedef _int_farptr_t       int_farptr_t;
typedef _uint_farptr_t      uint_farptr_t;
#endif

/* Greatest-width integer types */

typedef _intmax_t           intmax_t;
typedef _uintmax_t          uintmax_t;

#endif /* CONFIG_ARCH_STDINT_H */
#endif /* __INCLUDE_STDINT_H */
