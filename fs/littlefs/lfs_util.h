/****************************************************************************
 * fs/littlefs/lfs_util.h
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *
 * Ported by:
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: lihaichen <li8303@163.com>
 *
 * This port derives from ARM mbed logic which has a compatible 3-clause
 * BSD license:
 *
 *   Copyright (c) 2017, Arm Limited. All rights reserved.
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
 * 3. Neither the names ARM, NuttX nor the names of its contributors may be
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

#ifndef __FS_LITTLEFS_LFS_UTIL_H
#define __FS_LITTLEFS_LFS_UTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Users can override lfs_util.h with their own configuration by defining
 * LFS_CONFIG as a header file to include (-DLFS_CONFIG=lfs_config.h).
 *
 * If LFS_CONFIG is used, none of the default utils will be emitted and must be
 * provided by the config file. To start I would suggest copying lfs_util.h and
 * modifying as needed.
 */

#ifdef LFS_CONFIG
#  define LFS_STRINGIZE(x) LFS_STRINGIZE2(x)
#  define LFS_STRINGIZE2(x) #x
#  include LFS_STRINGIZE(LFS_CONFIG)
#else

/* System includes */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef LFS_NO_MALLOC
#  include <nuttx/kmalloc.h>
#endif
#ifndef LFS_NO_ASSERT
#  include <assert.h>
#endif
#if !defined(LFS_NO_DEBUG) || !defined(LFS_NO_WARN) || !defined(LFS_NO_ERROR)
#  include <debug.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros, may be replaced by system specific wrappers. Arguments to these
 * macros must not have side-effects as the macros can be removed for a smaller
 * code footprint
 */

/* Logging functions */

#ifndef LFS_NO_DEBUG
#  define LFS_DEBUG(fmt, ...) \
    finfo("lfs debug:%d: " fmt "\n", __LINE__, __VA_ARGS__)
#else
#  define LFS_DEBUG(fmt, ...)
#endif

#ifndef LFS_NO_WARN
#  define LFS_WARN(fmt, ...) \
    fwarn("lfs warn:%d: " fmt "\n", __LINE__, __VA_ARGS__)
#else
#  define LFS_WARN(fmt, ...)
#endif

#ifndef LFS_NO_ERROR
#  define LFS_ERROR(fmt, ...) \
    ferr("lfs error:%d: " fmt "\n", __LINE__, __VA_ARGS__)
#else
#  define LFS_ERROR(fmt, ...)
#endif

/* Runtime assertions */

#ifndef LFS_NO_ASSERT
#  define LFS_ASSERT(test) DEBUGASSERT(test)
#else
#  define LFS_ASSERT(test)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Builtin functions, these may be replaced by more efficient
 * toolchain-specific implementations. LFS_NO_INTRINSICS falls back to a more
 * expensive basic C implementation for debugging purposes
 */

/* Min/max functions for unsigned 32-bit numbers */

static inline uint32_t lfs_max(uint32_t a, uint32_t b)
{
  return (a > b) ? a : b;
}

static inline uint32_t lfs_min(uint32_t a, uint32_t b)
{
  return (a < b) ? a : b;
}

/* Find the next smallest power of 2 less than or equal to a */

static inline uint32_t lfs_npw2(uint32_t a)
{
#if !defined(LFS_NO_INTRINSICS) && (defined(__GNUC__) || defined(__CC_ARM))
  return 32 - __builtin_clz(a - 1);
#else
  uint32_t r = 0;
  uint32_t s;
  a -= 1;
  s = (a > 0xffff) << 4;
  a >>= s;
  r |= s;
  s = (a > 0xff) << 3;
  a >>= s;
  r |= s;
  s = (a > 0xf) << 2;
  a >>= s;
  r |= s;
  s = (a > 0x3) << 1;
  a >>= s;
  r |= s;
  return (r | (a >> 1)) + 1;
#endif
}

/* Count the number of trailing binary zeros in a
 * lfs_ctz(0) may be undefined
 */

static inline uint32_t lfs_ctz(uint32_t a)
{
#if !defined(LFS_NO_INTRINSICS) && defined(__GNUC__)
  return __builtin_ctz(a);
#else
  return lfs_npw2((a & -a) + 1) - 1;
#endif
}

/* Count the number of binary ones in a */

static inline uint32_t lfs_popc(uint32_t a)
{
#if !defined(LFS_NO_INTRINSICS) && (defined(__GNUC__) || defined(__CC_ARM))
  return __builtin_popcount(a);
#else
  a = a - ((a >> 1) & 0x55555555);
  a = (a & 0x33333333) + ((a >> 2) & 0x33333333);
  return (((a + (a >> 4)) & 0xf0f0f0f) * 0x1010101) >> 24;
#endif
}

/* Find the sequence comparison of a and b, this is the distance
 * between a and b ignoring overflow
 */

static inline int lfs_scmp(uint32_t a, uint32_t b)
{
  return (int)(unsigned)(a - b);
}

/* Convert from 32-bit little-endian to native order */

static inline uint32_t lfs_fromle32(uint32_t a)
{
#if !defined(CONFIG_ENDIAN_BIG)
  return a;
#elif !defined(LFS_NO_INTRINSICS)
  return __builtin_bswap32(a);
#else
  return (((uint8_t *)&a)[0] << 0) | (((uint8_t *)&a)[1] << 8) |
         (((uint8_t *)&a)[2] << 16) | (((uint8_t *)&a)[3] << 24);
#endif
}

/* Convert to 32-bit little-endian from native order */

static inline uint32_t lfs_tole32(uint32_t a)
{
  return lfs_fromle32(a);
}

/* Allocate memory, only used if buffers are not provided to littlefs */

static inline void *lfs_malloc(size_t size)
{
#ifndef LFS_NO_MALLOC
  return kmm_malloc(size);
#else
  return NULL;
#endif
}

/* Deallocate memory, only used if buffers are not provided to littlefs */

static inline void lfs_free(FAR void *p)
{
#ifndef LFS_NO_MALLOC
  kmm_free(p);
#else
  (void)p;
#endif
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 /* Calculate CRC-32 with polynomial = 0x04c11db7 */

void lfs_crc(uint32_t *crc, const void *buffer, size_t size);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
#endif /* __FS_LITTLEFS_LFS_UTIL_H */
