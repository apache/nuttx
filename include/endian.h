/****************************************************************************
 * include/endian.h
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

#ifndef __INCLUDE_ENDIAN_H
#define __INCLUDE_ENDIAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* The endian.h header must define at least the following macros for use in
 * determining host byte order for integer types:
 *
 *   BYTE_ORDER    This macro will have a value equal to one of the *_ENDIAN
 *                 macros in this header.
 *   LITTLE_ENDIAN If BYTE_ORDER == LITTLE_ENDIAN, the host byte order is
 *                 from least significant to most significant.
 *   BIG_ENDIAN    If BYTE_ORDER == BIG_ENDIAN, the host byte order is from
 *                 most significant to least significant.
 */

#define LITTLE_ENDIAN         1234
#define __LITTLE_ENDIAN       1234
#define BIG_ENDIAN            4321
#define __BIG_ENDIAN          4321

/* Common byte swapping macros */

#ifdef CONFIG_HAVE_BUILTIN_BSWAP16
#  define __swap_uint16 __builtin_bswap16
#else
#  define __swap_uint16(n) \
    (uint16_t)(((((uint16_t)(n)) & 0x00ff) << 8) | \
               ((((uint16_t)(n)) >> 8) & 0x00ff))
#endif

#ifdef CONFIG_HAVE_BUILTIN_BSWAP32
#  define __swap_uint32 __builtin_bswap32
#else
#  define __swap_uint32(n) \
    (uint32_t)(((((uint32_t)(n)) & 0x000000ffUL) << 24) | \
               ((((uint32_t)(n)) & 0x0000ff00UL) <<  8) | \
               ((((uint32_t)(n)) & 0x00ff0000UL) >>  8) | \
               ((((uint32_t)(n)) & 0xff000000UL) >> 24))
#endif

#ifdef CONFIG_HAVE_LONG_LONG
#  ifdef CONFIG_HAVE_BUILTIN_BSWAP64
#    define __swap_uint64 __builtin_bswap64
#  else
#    define __swap_uint64(n) \
        (uint64_t)(((((uint64_t)(n)) & 0x00000000000000ffULL) << 56) | \
                   ((((uint64_t)(n)) & 0x000000000000ff00ULL) << 40) | \
                   ((((uint64_t)(n)) & 0x0000000000ff0000ULL) << 24) | \
                   ((((uint64_t)(n)) & 0x00000000ff000000ULL) <<  8) | \
                   ((((uint64_t)(n)) & 0x000000ff00000000ULL) >>  8) | \
                   ((((uint64_t)(n)) & 0x0000ff0000000000ULL) >> 24) | \
                   ((((uint64_t)(n)) & 0x00ff000000000000ULL) >> 40) | \
                   ((((uint64_t)(n)) & 0xff00000000000000ULL) >> 56))
#  endif
#endif

/* Endian-specific definitions */

#ifdef CONFIG_ENDIAN_BIG
/* Big-endian byte order */

#  define BYTE_ORDER          BIG_ENDIAN
#  define __BYTE_ORDER        BIG_ENDIAN

/* Big-endian byte order macros */

#  define htobe16(n)          (n)
#  define htole16(n)          __swap_uint16((uint16_t)n)
#  define be16toh(n)          (n)
#  define le16toh(n)          __swap_uint16((uint16_t)n)

#  define htobe32(n)          (n)
#  define htole32(n)          __swap_uint32((uint32_t)n)
#  define be32toh(n)          (n)
#  define le32toh(n)          __swap_uint32(n)

#  ifdef CONFIG_HAVE_LONG_LONG
#    define htobe64(n)        (n)
#    define htole64(n)        __swap_uint64((uint64_t)n)
#    define be64toh(n)        (n)
#    define le64toh(n)        __swap_uint64((uint64_t)n)
#  endif

#else
/* Little-endian byte order */

#  define BYTE_ORDER          LITTLE_ENDIAN
#  define __BYTE_ORDER        __LITTLE_ENDIAN

/* Little-endian byte order macros */

#  define htobe16(n)          __swap_uint16((uint16_t)n)
#  define htole16(n)          (n)
#  define be16toh(n)          __swap_uint16((uint16_t)n)
#  define le16toh(n)          (n)

#  define htobe32(n)          __swap_uint32((uint32_t)n)
#  define htole32(n)          (n)
#  define be32toh(n)          __swap_uint32((uint32_t)n)
#  define le32toh(n)          (n)

#  ifdef CONFIG_HAVE_LONG_LONG
#    define htobe64(n)        __swap_uint64((uint64_t)n)
#    define htole64(n)        (n)
#    define be64toh(n)        __swap_uint64((uint64_t)n)
#    define le64toh(n)        (n)
#  endif
#endif
#endif /* __INCLUDE_ENDIAN_H */
