/****************************************************************************
 * include/nuttx/lib/stdbit.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LIB_STDBIT_H
#define __INCLUDE_NUTTX_LIB_STDBIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#ifdef CONFIG_ARCH_STDBIT_H
#  include <arch/stdbit.h>
#else

/****************************************************************************
 * Generic C23 stdbit implementation (CONFIG_LIBC_STDBIT_GENERIC).
 * Requires __builtin_clz, __builtin_ctz, __builtin_popcount (and ll
 * variants). If the toolchain does not provide them, enable the
 * corresponding CONFIG_HAVE_BUILTIN_* in nuttx/compiler.h or do not
 * use LIBC_STDBIT_GENERIC.
 ****************************************************************************/

#  if !defined(CONFIG_HAVE_BUILTIN_CLZ) || \
      !defined(CONFIG_HAVE_BUILTIN_CTZ) || \
      !defined(CONFIG_HAVE_BUILTIN_POPCOUNT)
#    error "Generic stdbit requires CONFIG_HAVE_BUILTIN_CLZ, CTZ, POPCOUNT"
#  endif

#  if defined(CONFIG_HAVE_LONG_LONG) && \
      !defined(CONFIG_HAVE_BUILTIN_POPCOUNTLL)
#    error "Generic stdbit 64-bit requires CONFIG_HAVE_BUILTIN_POPCOUNTLL"
#  endif

/****************************************************************************
 * C23 endianness macros
 ****************************************************************************/

#  ifdef CONFIG_ENDIAN_BIG
#    define __STDC_ENDIAN_NATIVE__  __STDC_ENDIAN_BIG__
#    define __STDC_ENDIAN_LITTLE__  1234
#    define __STDC_ENDIAN_BIG__     4321
#  else
#    define __STDC_ENDIAN_NATIVE__  __STDC_ENDIAN_LITTLE__
#    define __STDC_ENDIAN_LITTLE__  1234
#    define __STDC_ENDIAN_BIG__     4321
#  endif

/****************************************************************************
 * Leading / trailing zeros (C23: zero input returns bit width)
 ****************************************************************************/

#  define stdc_leading_zeros_uc(x) \
    ((unsigned char)(x) == 0 ? 8 : \
     (unsigned char)__builtin_clz((unsigned)(x) << 24))
#  define stdc_leading_zeros_us(x) \
    ((unsigned short)(x) == 0 ? 16 : \
     (unsigned short)__builtin_clz((unsigned)(x) << 16))
#  define stdc_leading_zeros_ui(x) \
    ((x) == 0 ? 32 : (unsigned)__builtin_clz(x))
#  define stdc_leading_zeros_ul(x) \
    ((x) == 0 ? (unsigned long)(8*sizeof(unsigned long)) : \
     (unsigned long)__builtin_clzl(x))
#  define stdc_leading_zeros_ull(x) \
    ((x) == 0 ? (unsigned long long)(8*sizeof(unsigned long long)) : \
     (unsigned long long)__builtin_clzll(x))

#  define stdc_trailing_zeros_uc(x) \
    ((unsigned char)(x) == 0 ? 8 : (unsigned char)__builtin_ctz((unsigned)(x)))
#  define stdc_trailing_zeros_us(x) \
    ((unsigned short)(x) == 0 ? 16 : \
     (unsigned short)__builtin_ctz((unsigned)(x)))
#  define stdc_trailing_zeros_ui(x) \
    ((x) == 0 ? 32 : (unsigned)__builtin_ctz(x))
#  define stdc_trailing_zeros_ul(x) \
    ((x) == 0 ? (unsigned long)(8*sizeof(unsigned long)) : \
     (unsigned long)__builtin_ctzl(x))
#  define stdc_trailing_zeros_ull(x) \
    ((x) == 0 ? (unsigned long long)(8*sizeof(unsigned long long)) : \
     (unsigned long long)__builtin_ctzll(x))

/****************************************************************************
 * Leading / trailing ones (C23: zero input returns 0)
 ****************************************************************************/

#  define stdc_leading_ones_uc(x)  stdc_leading_zeros_uc((unsigned char)~(x))
#  define stdc_leading_ones_us(x)  stdc_leading_zeros_us((unsigned short)~(x))
#  define stdc_leading_ones_ui(x)  stdc_leading_zeros_ui(~(x))
#  define stdc_leading_ones_ul(x)  stdc_leading_zeros_ul(~(x))
#  define stdc_leading_ones_ull(x) stdc_leading_zeros_ull(~(x))

#  define stdc_trailing_ones_uc(x)  stdc_trailing_zeros_uc((unsigned char)~(x))
#  define stdc_trailing_ones_us(x)  stdc_trailing_zeros_us((unsigned short)~(x))
#  define stdc_trailing_ones_ui(x)  stdc_trailing_zeros_ui(~(x))
#  define stdc_trailing_ones_ul(x)  stdc_trailing_zeros_ul(~(x))
#  define stdc_trailing_ones_ull(x) stdc_trailing_zeros_ull(~(x))

/****************************************************************************
 * First leading zero/one (bit index from MSB; C23: 0 returns bit width)
 ****************************************************************************/

#  define stdc_first_leading_zero_uc(x)  stdc_leading_zeros_uc(x)
#  define stdc_first_leading_zero_us(x) stdc_leading_zeros_us(x)
#  define stdc_first_leading_zero_ui(x) stdc_leading_zeros_ui(x)
#  define stdc_first_leading_zero_ul(x) stdc_leading_zeros_ul(x)
#  define stdc_first_leading_zero_ull(x) stdc_leading_zeros_ull(x)

#  define stdc_first_leading_one_uc(x)  stdc_leading_ones_uc(x)
#  define stdc_first_leading_one_us(x)  stdc_leading_ones_us(x)
#  define stdc_first_leading_one_ui(x)  stdc_leading_ones_ui(x)
#  define stdc_first_leading_one_ul(x)  stdc_leading_ones_ul(x)
#  define stdc_first_leading_one_ull(x) stdc_leading_ones_ull(x)

/****************************************************************************
 * First trailing zero/one (bit index from LSB; C23: 0 returns bit width)
 ****************************************************************************/

#  define stdc_first_trailing_zero_uc(x)  stdc_trailing_zeros_uc(x)
#  define stdc_first_trailing_zero_us(x)   stdc_trailing_zeros_us(x)
#  define stdc_first_trailing_zero_ui(x)   stdc_trailing_zeros_ui(x)
#  define stdc_first_trailing_zero_ul(x)   stdc_trailing_zeros_ul(x)
#  define stdc_first_trailing_zero_ull(x) stdc_trailing_zeros_ull(x)

#  define stdc_first_trailing_one_uc(x)   stdc_trailing_ones_uc(x)
#  define stdc_first_trailing_one_us(x)   stdc_trailing_ones_us(x)
#  define stdc_first_trailing_one_ui(x)   stdc_trailing_ones_ui(x)
#  define stdc_first_trailing_one_ul(x)   stdc_trailing_ones_ul(x)
#  define stdc_first_trailing_one_ull(x)  stdc_trailing_ones_ull(x)

/****************************************************************************
 * Count zeros / ones
 ****************************************************************************/

#  define stdc_count_ones_uc(x) \
    ((unsigned char)__builtin_popcount((unsigned)(x)))
#  define stdc_count_ones_us(x) \
    ((unsigned short)__builtin_popcount((unsigned)(x)))
#  define stdc_count_ones_ui(x)  ((unsigned)__builtin_popcount(x))
#  define stdc_count_ones_ul(x)  ((unsigned long)__builtin_popcountl(x))
#  define stdc_count_ones_ull(x) ((unsigned long long)__builtin_popcountll(x))

#  define stdc_count_zeros_uc(x)  ((unsigned char)8 - stdc_count_ones_uc(x))
#  define stdc_count_zeros_us(x)  ((unsigned short)16 - stdc_count_ones_us(x))
#  define stdc_count_zeros_ui(x)   (32 - stdc_count_ones_ui(x))
#  define stdc_count_zeros_ul(x)  \
    ((unsigned long)(8*sizeof(unsigned long)) - stdc_count_ones_ul(x))
#  define stdc_count_zeros_ull(x) \
    ((unsigned long long)(8*sizeof(unsigned long long)) - \
     stdc_count_ones_ull(x))

/****************************************************************************
 * Single-bit test, bit width, bit floor, bit ceil
 ****************************************************************************/

#  define stdc_has_single_bit_uc(x)  (stdc_count_ones_uc(x) == 1)
#  define stdc_has_single_bit_us(x)   (stdc_count_ones_us(x) == 1)
#  define stdc_has_single_bit_ui(x)   (stdc_count_ones_ui(x) == 1)
#  define stdc_has_single_bit_ul(x)   (stdc_count_ones_ul(x) == 1)
#  define stdc_has_single_bit_ull(x)  (stdc_count_ones_ull(x) == 1)

#  define stdc_bit_width_uc(x) \
    ((unsigned char)(x) == 0 ? 0 : 9 - stdc_leading_zeros_uc(x))
#  define stdc_bit_width_us(x) \
    ((unsigned short)(x) == 0 ? 0 : 17 - stdc_leading_zeros_us(x))
#  define stdc_bit_width_ui(x)  ((x) == 0 ? 0 : 33 - stdc_leading_zeros_ui(x))
#  define stdc_bit_width_ul(x) \
    ((x) == 0 ? 0 : (int)(8*sizeof(unsigned long) + 1 - \
     stdc_leading_zeros_ul(x)))
#  define stdc_bit_width_ull(x) \
    ((x) == 0 ? 0 : (int)(8*sizeof(unsigned long long) + 1 - \
     stdc_leading_zeros_ull(x)))

#  define stdc_bit_floor_uc(x) \
    ((unsigned char)((x) != 0 ? 1 << (7 - stdc_leading_zeros_uc(x)) : 0))
#  define stdc_bit_floor_us(x) \
    ((unsigned short)((x) != 0 ? 1 << (15 - stdc_leading_zeros_us(x)) : 0))
#  define stdc_bit_floor_ui(x) \
    ((x) != 0 ? 1u << (31 - stdc_leading_zeros_ui(x)) : 0u)
#  define stdc_bit_floor_ul(x) \
    ((x) != 0 ? (unsigned long)1 << (8*sizeof(unsigned long)-1 - \
     stdc_leading_zeros_ul(x)) : 0ul)
#  define stdc_bit_floor_ull(x) \
    ((x) != 0 ? (unsigned long long)1 << (8*sizeof(unsigned long long)-1 - \
     stdc_leading_zeros_ull(x)) : 0ull)

#  define stdc_bit_ceil_uc(x) \
    ((unsigned char)((x) <= 1 ? 1 : (unsigned char)1 << (9 - \
     stdc_leading_zeros_uc((unsigned char)(x)-1))))
#  define stdc_bit_ceil_us(x) \
    ((unsigned short)((x) <= 1 ? 1 : (unsigned short)1 << (17 - \
     stdc_leading_zeros_us((unsigned short)(x)-1))))
#  define stdc_bit_ceil_ui(x) \
    ((x) <= 1 ? 1u : 1u << (33 - stdc_leading_zeros_ui((x)-1)))
#  define stdc_bit_ceil_ul(x) \
    ((x) <= 1 ? 1ul : 1ul << (8*sizeof(unsigned long) + 1 - \
     stdc_leading_zeros_ul((x)-1)))
#  define stdc_bit_ceil_ull(x) \
    ((x) <= 1 ? 1ull : 1ull << (8*sizeof(unsigned long long) + 1 - \
     stdc_leading_zeros_ull((x)-1)))

#endif /* CONFIG_ARCH_STDBIT_H */

#endif /* __INCLUDE_NUTTX_LIB_STDBIT_H */
