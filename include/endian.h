/****************************************************************************
 * include/endian.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#define BIG_ENDIAN            4321

/* Common byte swapping macros */

#define __SWAP_UINT16_ISMACRO 1
#undef  __SWAP_UINT32_ISMACRO

#ifdef __SWAP_UINT16_ISMACRO
#  define __swap_uint16(n) \
    (uint16_t)(((((uint16_t)(n)) & 0x00ff) << 8) | \
               ((((uint16_t)(n)) >> 8) & 0x00ff))
#endif

#ifdef __SWAP_UINT32_ISMACRO
#  define __swap_uint32(n) \
    (uint32_t)(((((uint32_t)(n)) & 0x000000ffUL) << 24) | \
               ((((uint32_t)(n)) & 0x0000ff00UL) <<  8) | \
               ((((uint32_t)(n)) & 0x00ff0000UL) >>  8) | \
               ((((uint32_t)(n)) & 0xff000000UL) >> 24))
#endif

/* Endian-specific definitions */

#ifdef CONFIG_ENDIAN_BIG
/* Big-endian byte order */

#  define BYTE_ORDER          BIG_ENDIAN

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __SWAP_UINT16_ISMACRO
uint16_t __swap_uint16(uint16_t n);
#endif

#ifndef __SWAP_UINT32_ISMACRO
uint32_t __swap_uint32(uint32_t n);
#endif

#if CONFIG_HAVE_LONG_LONG
uint64_t __swap_uint64(uint64_t n);
#endif

#endif /* __INCLUDE_ENDIAN_H */
