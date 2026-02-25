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

#ifdef CONFIG_ARCH_STDBIT_H
#  include <arch/stdbit.h>
#else

#  include <nuttx/compiler.h>
#  include <stdint.h>

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
 * C23 endianness macros (reuse LITTLE_ENDIAN/BIG_ENDIAN from sys/endian.h)
 ****************************************************************************/

#  include <sys/endian.h>
#  define __STDC_ENDIAN_LITTLE__  LITTLE_ENDIAN
#  define __STDC_ENDIAN_BIG__     BIG_ENDIAN
#  define __STDC_ENDIAN_NATIVE__  BYTE_ORDER

/****************************************************************************
 * Leading / trailing zeros (C23: zero input returns bit width)
 ****************************************************************************/

static inline unsigned int stdc_leading_zeros_uc(unsigned char x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned char)) :
         __builtin_clz((unsigned)x << (8*sizeof(unsigned) - 8)));
}

static inline unsigned int stdc_leading_zeros_us(unsigned short x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned short)) :
         __builtin_clz((unsigned)x <<
         (8*sizeof(unsigned) - 16)));
}

static inline unsigned int stdc_leading_zeros_ui(unsigned int x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned int)) :
         __builtin_clz(x));
}

static inline unsigned int stdc_leading_zeros_ul(unsigned long x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned long)) :
         __builtin_clzl(x));
}

static inline unsigned int stdc_leading_zeros_ull(unsigned long long x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned long long)) :
         __builtin_clzll(x));
}

static inline unsigned int stdc_trailing_zeros_uc(unsigned char x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned char)) :
         __builtin_ctz((unsigned)x));
}

static inline unsigned int stdc_trailing_zeros_us(unsigned short x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned short)) :
         __builtin_ctz((unsigned)x));
}

static inline unsigned int stdc_trailing_zeros_ui(unsigned int x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned int)) :
         __builtin_ctz(x));
}

static inline unsigned int stdc_trailing_zeros_ul(unsigned long x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned long)) :
         __builtin_ctzl(x));
}

static inline unsigned int stdc_trailing_zeros_ull(unsigned long long x)
{
  return (unsigned int)(x == 0 ? (8*sizeof(unsigned long long)) :
         __builtin_ctzll(x));
}

/****************************************************************************
 * Leading / trailing ones (C23: zero input returns 0)
 ****************************************************************************/

#  define stdc_leading_ones_uc(x)  stdc_leading_zeros_uc((unsigned char)~(x))
#  define stdc_leading_ones_us(x)  stdc_leading_zeros_us((unsigned short)~(x))
#  define stdc_leading_ones_ui(x)  stdc_leading_zeros_ui(~(x))
#  define stdc_leading_ones_ul(x)  stdc_leading_zeros_ul(~(x))
#  define stdc_leading_ones_ull(x) stdc_leading_zeros_ull(~(x))

#  define stdc_trailing_ones_uc(x)  \
    stdc_trailing_zeros_uc((unsigned char)~(x))
#  define stdc_trailing_ones_us(x)  \
    stdc_trailing_zeros_us((unsigned short)~(x))
#  define stdc_trailing_ones_ui(x)  stdc_trailing_zeros_ui(~(x))
#  define stdc_trailing_ones_ul(x)  stdc_trailing_zeros_ul(~(x))
#  define stdc_trailing_ones_ull(x) stdc_trailing_zeros_ull(~(x))

/****************************************************************************
 * First leading zero/one (bit index from MSB; C23: 0 returns bit width)
 ****************************************************************************/

static inline unsigned int stdc_first_leading_zero_uc(unsigned char x)
{
  return (unsigned int)(x == (unsigned char)0xff ? 0 :
         stdc_leading_ones_uc(x) + 1);
}

static inline unsigned int stdc_first_leading_zero_us(unsigned short x)
{
  return (unsigned int)(x == (unsigned short)0xffff ? 0 :
         stdc_leading_ones_us(x) + 1);
}

static inline unsigned int stdc_first_leading_zero_ui(unsigned int x)
{
  return (unsigned int)(x == ~0u ? 0 : stdc_leading_ones_ui(x) + 1);
}

static inline unsigned int stdc_first_leading_zero_ul(unsigned long x)
{
  return (unsigned int)(x == ~0ul ? 0 : stdc_leading_ones_ul(x) + 1);
}

static inline unsigned int stdc_first_leading_zero_ull(unsigned long long x)
{
  return (unsigned int)(x == ~0ull ? 0 : stdc_leading_ones_ull(x) + 1);
}

static inline unsigned int stdc_first_leading_one_uc(unsigned char x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_leading_zeros_uc(x) + 1);
}

static inline unsigned int stdc_first_leading_one_us(unsigned short x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_leading_zeros_us(x) + 1);
}

static inline unsigned int stdc_first_leading_one_ui(unsigned int x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_leading_zeros_ui(x) + 1);
}

static inline unsigned int stdc_first_leading_one_ul(unsigned long x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_leading_zeros_ul(x) + 1);
}

static inline unsigned int stdc_first_leading_one_ull(unsigned long long x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_leading_zeros_ull(x) + 1);
}

/****************************************************************************
 * First trailing zero/one (bit index from LSB; C23: 0 returns bit width)
 ****************************************************************************/

static inline unsigned int stdc_first_trailing_zero_uc(unsigned char x)
{
  return (unsigned int)(x == (unsigned char)0xff ? 0 :
         stdc_trailing_ones_uc(x) + 1);
}

static inline unsigned int stdc_first_trailing_zero_us(unsigned short x)
{
  return (unsigned int)(x == (unsigned short)0xffff ? 0 :
         stdc_trailing_ones_us(x) + 1);
}

static inline unsigned int stdc_first_trailing_zero_ui(unsigned int x)
{
  return (unsigned int)(x == ~0u ? 0 : stdc_trailing_ones_ui(x) + 1);
}

static inline unsigned int stdc_first_trailing_zero_ul(unsigned long x)
{
  return (unsigned int)(x == ~0ul ? 0 : stdc_trailing_ones_ul(x) + 1);
}

static inline unsigned int stdc_first_trailing_zero_ull(unsigned long long x)
{
  return (unsigned int)(x == ~0ull ? 0 : stdc_trailing_ones_ull(x) + 1);
}

static inline unsigned int stdc_first_trailing_one_uc(unsigned char x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_trailing_zeros_uc(x) + 1);
}

static inline unsigned int stdc_first_trailing_one_us(unsigned short x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_trailing_zeros_us(x) + 1);
}

static inline unsigned int stdc_first_trailing_one_ui(unsigned int x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_trailing_zeros_ui(x) + 1);
}

static inline unsigned int stdc_first_trailing_one_ul(unsigned long x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_trailing_zeros_ul(x) + 1);
}

static inline unsigned int stdc_first_trailing_one_ull(unsigned long long x)
{
  return (unsigned int)(x == 0 ? 0 : stdc_trailing_zeros_ull(x) + 1);
}

/****************************************************************************
 * Count zeros / ones
 ****************************************************************************/

#  define stdc_count_ones_uc(x) \
    ((unsigned int)__builtin_popcount((unsigned)(x)))
#  define stdc_count_ones_us(x) \
    ((unsigned int)__builtin_popcount((unsigned)(x)))
#  define stdc_count_ones_ui(x)  ((unsigned int)__builtin_popcount(x))
#  define stdc_count_ones_ul(x)  ((unsigned int)__builtin_popcountl(x))
#  define stdc_count_ones_ull(x) ((unsigned int)__builtin_popcountll(x))

#  define stdc_count_zeros_uc(x) \
    ((unsigned int)((8*sizeof(unsigned char)) - stdc_count_ones_uc(x)))
#  define stdc_count_zeros_us(x) \
    ((unsigned int)((8*sizeof(unsigned short)) - stdc_count_ones_us(x)))
#  define stdc_count_zeros_ui(x) \
    ((unsigned int)((8*sizeof(unsigned int)) - stdc_count_ones_ui(x)))
#  define stdc_count_zeros_ul(x)  \
    ((unsigned int)((8*sizeof(unsigned long)) - stdc_count_ones_ul(x)))
#  define stdc_count_zeros_ull(x) \
    ((unsigned int)((8*sizeof(unsigned long long)) - stdc_count_ones_ull(x)))

/****************************************************************************
 * Single-bit test, bit width, bit floor, bit ceil
 ****************************************************************************/

#  define stdc_has_single_bit_uc(x)  (stdc_count_ones_uc(x) == 1)
#  define stdc_has_single_bit_us(x)   (stdc_count_ones_us(x) == 1)
#  define stdc_has_single_bit_ui(x)   (stdc_count_ones_ui(x) == 1)
#  define stdc_has_single_bit_ul(x)   (stdc_count_ones_ul(x) == 1)
#  define stdc_has_single_bit_ull(x)  (stdc_count_ones_ull(x) == 1)

#  define stdc_bit_width_uc(x) \
    ((unsigned int)((unsigned char)(x) == 0 ? 0 : \
    8 - stdc_leading_zeros_uc(x)))
#  define stdc_bit_width_us(x) \
    ((unsigned int)((unsigned short)(x) == 0 ? 0 : \
    16 - stdc_leading_zeros_us(x)))
#  define stdc_bit_width_ui(x) \
    ((unsigned int)((x) == 0 ? 0 : \
    (8*sizeof(unsigned int)) - stdc_leading_zeros_ui(x)))
#  define stdc_bit_width_ul(x) \
    ((unsigned int)((x) == 0 ? 0 : \
    (8*sizeof(unsigned long) - stdc_leading_zeros_ul(x))))
#  define stdc_bit_width_ull(x) \
    ((unsigned int)((x) == 0 ? 0 : \
    (8*sizeof(unsigned long long) - stdc_leading_zeros_ull(x))))

/****************************************************************************
 * bit_floor / bit_ceil: inline functions to avoid double evaluation of x
 ****************************************************************************/

static inline unsigned char stdc_bit_floor_uc(unsigned char x)
{
  return (unsigned char)(x != 0 ? 1 << ((8*sizeof(unsigned char) - 1) -
              stdc_leading_zeros_uc(x)) : 0);
}

static inline unsigned short stdc_bit_floor_us(unsigned short x)
{
  return (unsigned short)(x != 0 ? 1 << ((8*sizeof(unsigned short) - 1) -
              stdc_leading_zeros_us(x)) : 0);
}

static inline unsigned int stdc_bit_floor_ui(unsigned int x)
{
  return x != 0 ? 1u << ((8*sizeof(unsigned int) - 1) -
              stdc_leading_zeros_ui(x)) : 0u;
}

static inline unsigned long stdc_bit_floor_ul(unsigned long x)
{
  return x != 0 ? (unsigned long)1 << ((8*sizeof(unsigned long) - 1) -
              stdc_leading_zeros_ul(x)) : 0ul;
}

static inline unsigned long long stdc_bit_floor_ull(unsigned long long x)
{
  return x != 0 ? (unsigned long long)1 << ((8*sizeof(unsigned long long)
         - 1) - stdc_leading_zeros_ull(x)) : 0ull;
}

static inline unsigned char stdc_bit_ceil_uc(unsigned char x)
{
  return (unsigned char)(x <= 1 ? 1 : (unsigned char)1 <<
         ((8*sizeof(unsigned char)) -
          stdc_leading_zeros_uc((unsigned char)(x - 1))));
}

static inline unsigned short stdc_bit_ceil_us(unsigned short x)
{
  return (unsigned short)(x <= 1 ? 1 : (unsigned short)1 <<
         ((8*sizeof(unsigned short)) -
          stdc_leading_zeros_us((unsigned short)(x - 1))));
}

static inline unsigned int stdc_bit_ceil_ui(unsigned int x)
{
  return x <= 1 ? 1u : 1u << ((8*sizeof(unsigned int)) -
         stdc_leading_zeros_ui(x - 1));
}

static inline unsigned long stdc_bit_ceil_ul(unsigned long x)
{
  return x <= 1 ? 1ul : 1ul << ((8*sizeof(unsigned long)) -
         stdc_leading_zeros_ul(x - 1));
}

static inline unsigned long long stdc_bit_ceil_ull(unsigned long long x)
{
  return x <= 1 ? 1ull : 1ull << ((8*sizeof(unsigned long long)) -
         stdc_leading_zeros_ull(x - 1));
}

/****************************************************************************
 * C23 type-generic macros (dispatch via _Generic)
 ****************************************************************************/

#  if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L

#    define stdc_leading_zeros(x) _Generic((x), \
        unsigned char: stdc_leading_zeros_uc, \
        unsigned short: stdc_leading_zeros_us, \
        unsigned int: stdc_leading_zeros_ui, \
        unsigned long: stdc_leading_zeros_ul, \
        unsigned long long: stdc_leading_zeros_ull)(x)

#    define stdc_leading_ones(x) _Generic((x), \
        unsigned char: stdc_leading_ones_uc, \
        unsigned short: stdc_leading_ones_us, \
        unsigned int: stdc_leading_ones_ui, \
        unsigned long: stdc_leading_ones_ul, \
        unsigned long long: stdc_leading_ones_ull)(x)

#    define stdc_trailing_zeros(x) _Generic((x), \
        unsigned char: stdc_trailing_zeros_uc, \
        unsigned short: stdc_trailing_zeros_us, \
        unsigned int: stdc_trailing_zeros_ui, \
        unsigned long: stdc_trailing_zeros_ul, \
        unsigned long long: stdc_trailing_zeros_ull)(x)

#    define stdc_trailing_ones(x) _Generic((x), \
        unsigned char: stdc_trailing_ones_uc, \
        unsigned short: stdc_trailing_ones_us, \
        unsigned int: stdc_trailing_ones_ui, \
        unsigned long: stdc_trailing_ones_ul, \
        unsigned long long: stdc_trailing_ones_ull)(x)

#    define stdc_first_leading_zero(x) _Generic((x), \
        unsigned char: stdc_first_leading_zero_uc, \
        unsigned short: stdc_first_leading_zero_us, \
        unsigned int: stdc_first_leading_zero_ui, \
        unsigned long: stdc_first_leading_zero_ul, \
        unsigned long long: stdc_first_leading_zero_ull)(x)

#    define stdc_first_leading_one(x) _Generic((x), \
        unsigned char: stdc_first_leading_one_uc, \
        unsigned short: stdc_first_leading_one_us, \
        unsigned int: stdc_first_leading_one_ui, \
        unsigned long: stdc_first_leading_one_ul, \
        unsigned long long: stdc_first_leading_one_ull)(x)

#    define stdc_first_trailing_zero(x) _Generic((x), \
        unsigned char: stdc_first_trailing_zero_uc, \
        unsigned short: stdc_first_trailing_zero_us, \
        unsigned int: stdc_first_trailing_zero_ui, \
        unsigned long: stdc_first_trailing_zero_ul, \
        unsigned long long: stdc_first_trailing_zero_ull)(x)

#    define stdc_first_trailing_one(x) _Generic((x), \
        unsigned char: stdc_first_trailing_one_uc, \
        unsigned short: stdc_first_trailing_one_us, \
        unsigned int: stdc_first_trailing_one_ui, \
        unsigned long: stdc_first_trailing_one_ul, \
        unsigned long long: stdc_first_trailing_one_ull)(x)

#    define stdc_count_zeros(x) _Generic((x), \
        unsigned char: stdc_count_zeros_uc, \
        unsigned short: stdc_count_zeros_us, \
        unsigned int: stdc_count_zeros_ui, \
        unsigned long: stdc_count_zeros_ul, \
        unsigned long long: stdc_count_zeros_ull)(x)

#    define stdc_count_ones(x) _Generic((x), \
        unsigned char: stdc_count_ones_uc, \
        unsigned short: stdc_count_ones_us, \
        unsigned int: stdc_count_ones_ui, \
        unsigned long: stdc_count_ones_ul, \
        unsigned long long: stdc_count_ones_ull)(x)

#    define stdc_has_single_bit(x) _Generic((x), \
        unsigned char: stdc_has_single_bit_uc, \
        unsigned short: stdc_has_single_bit_us, \
        unsigned int: stdc_has_single_bit_ui, \
        unsigned long: stdc_has_single_bit_ul, \
        unsigned long long: stdc_has_single_bit_ull)(x)

#    define stdc_bit_width(x) _Generic((x), \
        unsigned char: stdc_bit_width_uc, \
        unsigned short: stdc_bit_width_us, \
        unsigned int: stdc_bit_width_ui, \
        unsigned long: stdc_bit_width_ul, \
        unsigned long long: stdc_bit_width_ull)(x)

#    define stdc_bit_floor(x) _Generic((x), \
        unsigned char: stdc_bit_floor_uc, \
        unsigned short: stdc_bit_floor_us, \
        unsigned int: stdc_bit_floor_ui, \
        unsigned long: stdc_bit_floor_ul, \
        unsigned long long: stdc_bit_floor_ull)(x)

#    define stdc_bit_ceil(x) _Generic((x), \
        unsigned char: stdc_bit_ceil_uc, \
        unsigned short: stdc_bit_ceil_us, \
        unsigned int: stdc_bit_ceil_ui, \
        unsigned long: stdc_bit_ceil_ul, \
        unsigned long long: stdc_bit_ceil_ull)(x)

#  endif /* __STDC_VERSION__ >= 201112L */

#endif /* CONFIG_ARCH_STDBIT_H */

#endif /* __INCLUDE_NUTTX_LIB_STDBIT_H */
