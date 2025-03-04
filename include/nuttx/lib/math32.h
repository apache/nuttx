/****************************************************************************
 * include/nuttx/lib/math32.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_LIB_MATH32_H
#define __INCLUDE_NUTTX_LIB_MATH32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <strings.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define div_round_up(n, d)      (((n) + (d) - 1) / (d))
#define div_round_closest(n, d) ((((n) < 0) ^ ((d) < 0)) ? \
                                (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))

/* Returns one plus the index of the most significant 1-bit of n,
 * or if n is zero, returns zero.
 */

#if UINTPTR_MAX > UINT32_MAX
#  define FLS(n) ((n) & UINT64_C(0xffffffff00000000) ? 32 + \
                  FLS32((size_t)(n) >> 32) : FLS32(n))
#else
#  define FLS(n) FLS32(n)
#endif

#define FLS32(n) ((n) & 0xffff0000 ? 16 + FLS16((n) >> 16) : FLS16(n))
#define FLS16(n) ((n) & 0xff00     ?  8 + FLS8 ((n) >>  8) : FLS8 (n))
#define FLS8(n)  ((n) & 0xf0       ?  4 + FLS4 ((n) >>  4) : FLS4 (n))
#define FLS4(n)  ((n) & 0xc        ?  2 + FLS2 ((n) >>  2) : FLS2 (n))
#define FLS2(n)  ((n) & 0x2        ?  1 + FLS1 ((n) >>  1) : FLS1 (n))
#define FLS1(n)  ((n) & 0x1        ?  1 : 0)

/* Checks if an integer is power of two at compile time */

#define IS_POWER_OF_2(n)           ((n) > 0 && ((n) & (n - 1)) == 0)

/* Returns round up and round down value of log2(n). Note: it can be used at
 * compile time.
 */

#define LOG2_CEIL(n)  (IS_POWER_OF_2(n) ? FLS(n) - 1 : FLS(n))
#define LOG2_FLOOR(n) (FLS(n) - 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These types are useful on platforms that do not support 64-bit types. */

struct int64_s
{
#ifdef CONFIG_ENDIAN_BIG
  int32_t  ms;
  uint32_t ls;
#else
  uint32_t ls;
  int32_t  ms;
#endif
};

struct uint64_s
{
#ifdef CONFIG_ENDIAN_BIG
  uint32_t ms;
  uint32_t ls;
#else
  uint32_t ls;
  uint32_t ms;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#define flsx(n) ((sizeof(n) <= sizeof(long)) ? flsl(n) : flsll(n))

/****************************************************************************
 * Name: log2ceil
 *
 * Description:
 *   Calculate the up-rounded power-of-two for input.
 *
 * Input Parameters:
 *   x - Argument to calculate the power-of-two from.
 *
 * Returned Value:
 *   Power-of-two for argument, rounded up.
 *
 ****************************************************************************/

#define log2ceil(n) (IS_POWER_OF_2(n) ? (flsx(n) - 1) : flsx(n))

/****************************************************************************
 * Name: log2floor
 *
 * Description:
 *   Calculate the down-rounded (truncated) power-of-two for input.
 *
 * Input Parameters:
 *   x - Argument to calculate the power-of-two from.
 *
 * Returned Value:
 *   Power-of-two for argument, rounded (truncated) down.
 *
 ****************************************************************************/

#define log2floor(n) (flsx(n) - 1)

/* roundup_pow_of_two() - Round up to nearest power of two
 * n: value to round up
 */

#define roundup_pow_of_two(n) (((n) - (n) + 1) << flsx((n) - 1))

/* rounddown_pow_of_two() - Round down to nearest power of two
 * n: value to round down
 */

#define rounddown_pow_of_two(n) (((n) - (n) + 1) << (flsx(n) - 1))

/* order_base_2 - Calculate the (rounded up) base 2 order of the argument
 * n: parameter
 *
 * The first few values calculated by this routine:
 *  ob2(0) = 0
 *  ob2(1) = 0
 *  ob2(2) = 1
 *  ob2(3) = 2
 *  ob2(4) = 2
 *  ob2(5) = 3
 *  ... and so on.
 */

#define order_base_2(n) ((n) > 1 ? log2floor((n) - 1) + 1 : 0)

/* If the divisor happens to be constant, we determine the appropriate
 * inverse at compile time to turn the division into a few inline
 * multiplications which ought to be much faster.
 *
 * (It is unfortunate that gcc doesn't perform all this internally.)
 */

#ifdef CONFIG_HAVE_LONG_LONG
/* Default C implementation for umul64_const()
 *
 * Prototype: uint64_t umul64_const(uint64_t retval, uint64_t m,
 *                                  uint64_t n, bool bias);
 * Semantic:  retval = ((bias ? m : 0) + m * n) >> 64
 *
 * The product is a 128-bit value, scaled down to 64 bits.
 * Assuming constant propagation to optimize away unused conditional code.
 * Architectures may provide their own optimized assembly implementation.
 */

#  ifdef up_umul64_const
#    define umul64_const(res, m, n, bias) \
      (res) = up_umul64_const(m, n, bias)
#  else
#    define umul64_const(res, m, n, bias) \
      do \
        { \
          uint32_t __m_lo = (m) & 0xffffffff; \
          uint32_t __m_hi = (m) >> 32; \
          uint32_t __n_lo = (n) & 0xffffffff; \
          uint32_t __n_hi = (n) >> 32; \
          uint32_t __res_lo; \
          uint32_t __res_hi; \
          uint32_t __tmp; \
          \
          if (!(bias)) \
            { \
              (res) = ((uint64_t)__m_lo * __n_lo) >> 32; \
            } \
          else if (!((m) & ((1ULL << 63) | (1ULL << 31)))) \
            { \
              (res) = ((m) + (uint64_t)__m_lo * __n_lo) >> 32; \
            } \
          else \
            { \
              (res) = (m) + (uint64_t)__m_lo * __n_lo; \
              __res_lo = (res) >> 32; \
              __res_hi = (__res_lo < __m_hi); \
              (res) = __res_lo | ((uint64_t)__res_hi << 32); \
            } \
          \
          if (!((m) & ((1ULL << 63) | (1ULL << 31)))) \
            { \
              (res) += (uint64_t)__m_lo * __n_hi; \
              (res) += (uint64_t)__m_hi * __n_lo; \
              (res) >>= 32; \
            } \
          else \
            { \
              (res) += (uint64_t)__m_lo * __n_hi; \
              __tmp = (res) >> 32; \
              (res) += (uint64_t)__m_hi * __n_lo; \
              __res_lo = (res) >> 32; \
              __res_hi = (__res_lo < __tmp); \
              (res) = __res_lo | ((uint64_t)__res_hi << 32); \
            } \
          \
          (res) += (uint64_t)__m_hi * __n_hi; \
        } \
      while (0)
#  endif

#  define div64_const32(n, b) \
    do \
      { \
        uint64_t ___res; \
        uint64_t ___x; \
        uint64_t ___t; \
        uint64_t ___m; \
        uint64_t ___n = (n); \
        uint32_t ___p; \
        uint32_t ___bias; \
        uint32_t ___b = (b); \
        ___p = 1 << LOG2_FLOOR(___b); \
        ___m = (~0ULL / ___b) * ___p; \
        ___m += (((~0ULL % ___b + 1) * ___p) + ___b - 1) / ___b; \
        ___x = ~0ULL / ___b * ___b - 1; \
        ___res = ((___m & 0xffffffff) * (___x & 0xffffffff)) >> 32; \
        ___t = ___res += (___m & 0xffffffff) * (___x >> 32); \
        ___res += (___x & 0xffffffff) * (___m >> 32); \
        ___t = (___res < ___t) ? (1ULL << 32) : 0; \
        ___res = (___res >> 32) + ___t; \
        ___res += (___m >> 32) * (___x >> 32); \
        ___res /= ___p; \
        if (~0ULL % (___b / (___b & -___b)) == 0) \
          { \
            ___n /= (___b & -___b); \
            ___m = ~0ULL / (___b / (___b & -___b)); \
            ___p = 1; \
            ___bias = 1; \
          } \
        else if (___res != ___x / ___b) \
          { \
            ___bias = 1; \
            ___m = (~0ULL / ___b) * ___p; \
            ___m += ((~0ULL % ___b + 1) * ___p) / ___b; \
          } \
        else \
          { \
            uint32_t ___bits = -(___m & -___m); \
            ___bits |= ___m >> 32; \
            ___bits = (~___bits) << 1; \
            if (!___bits) \
              { \
                ___p /= (___m & -___m); \
                ___m /= (___m & -___m); \
              } \
            else \
              { \
                ___p >>= LOG2_FLOOR(___bits); \
                ___m >>= LOG2_FLOOR(___bits); \
              } \
            ___bias = 0; \
          } \
        umul64_const(___res, ___m, ___n, ___bias); \
        \
        ___res /= ___p; \
        (n) = ___res; \
      } \
    while (0)

#endif

#if defined(CONFIG_HAVE_LONG_LONG) && defined(CONFIG_HAVE_EXPRESSION_STATEMENT)
#  define div64_const(n, base) \
    ({ \
      uint64_t __n = (n); \
      uint32_t __base = (base); \
      if (IS_POWER_OF_2(__base)) \
        { \
          (__n) >>= LOG2_FLOOR(__base); \
        } \
      else if (UINTPTR_MAX == UINT32_MAX) \
        { \
          div64_const32(__n, __base); \
        } \
      else \
        { \
          __n /= __base; \
        } \
        __n; \
      })

#  define div_const(n, base) \
    ((sizeof(typeof(n)) == sizeof(uint64_t)) ? div64_const(n, base) : ((n) / (base)))
#  define div_const_roundup(n, base) \
    ((sizeof(typeof(n)) == sizeof(uint64_t)) ? div64_const((n) + (base) - 1, base) : \
     (((n) + (base) - 1) / (base)))
#  define div_const_roundnearest(n, base) \
    ((sizeof(typeof(n)) == sizeof(uint64_t)) ? div64_const((n) + ((base) / 2), base) : \
     (((n) + ((base) / 2)) / (base)))
#else
#  define div_const(n, base) ((n) / (base))
#  define div_const_roundup(n, base) (((n) + (base) - 1) / (base))
#  define div_const_roundnearest(n, base) (((n) + ((base) / 2)) / (base))
#endif

/****************************************************************************
 * Name: uneg64
 *
 * Description:
 *   Negate a 64-bit unsigned value.
 *
 * Input Parameters:
 *   value - The value to be negated.
 *
 ****************************************************************************/

/* void uneg64(FAR const uint64_s *value); */

#define uneg64(value) \
  do \
    { \
      value->ms = ~value->ms; \
      value->ls = -value->ls; \
      if (value->ls == 0) \
        { \
          value->ms++; \
        } \
    } \
  while (0)

/****************************************************************************
 * Name: uadd32x64
 *
 * Description:
 *   Add a 32-bit value to a 64-bit values and return the truncated 64-bit
 *   sum.
 *
 * Input Parameters:
 *   term1 and term2 - The values to be added
 *   sum - The location to return the product of the two values.  sum may
 *     be one of term1 or term2
 *
 ****************************************************************************/

void uadd32x64(uint32_t term1, FAR const struct uint64_s *term2,
               FAR struct uint64_s *sum);

/****************************************************************************
 * Name: uadd64
 *
 * Description:
 *   Add two 64-bit values and return a 64-bit sum.
 *
 * Input Parameters:
 *   term1 and term2 - The values to be added
 *   sum - The location to return the product of the two values.  sum may
 *     be one of term1 or term2
 *
 ****************************************************************************/

void uadd64(FAR const struct uint64_s *term1,
            FAR const struct uint64_s *term2,
            FAR struct uint64_s *sum);

/****************************************************************************
 * Name: usub64x32
 *
 * Description:
 *   Subtract a 32-bit value from a 64-bit value and return the 64-bit
 *   difference.
 *
 * Input Parameters:
 *   minuend    - The number from which another number (the Subtrahend) is
 *     to be subtracted.
 *   subtrahend - The number that is to be subtracted.
 *   difference - The location to return the difference of the two values.
 *     difference may the same as one of minuend or subtrahend.
 *
 ****************************************************************************/

void usub64x32(FAR const struct uint64_s *minuend, uint32_t subtrahend,
               FAR struct uint64_s *difference);

/****************************************************************************
 * Name: usub64
 *
 * Description:
 *   Subtract two 64-bit values and return the 64-bit difference.
 *
 * Input Parameters:
 *   minuend    - The number from which another number (the Subtrahend) is
 *     to be subtracted.
 *   subtrahend - The number that is to be subtracted.
 *   difference - The location to return the difference of the two values.
 *     difference may the same as one of minuend or subtrahend.
 *
 ****************************************************************************/

void usub64(FAR const struct uint64_s *minuend,
            FAR const struct uint64_s *subtrahend,
            FAR struct uint64_s *difference);

/****************************************************************************
 * Name: umul32
 *
 * Description:
 *   Multiply two 32-bit values, factor1 and factor2, and return the
 *   full 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul32(uint32_t factor1, uint32_t factor2,
            FAR struct uint64_s *product);

/****************************************************************************
 * Name: umul32x64
 *
 * Description:
 *   Multiply one 32-bit and one 64-bit values, factor1 and factor2,
 *   respectively, and return the truncated 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul32x64(uint32_t factor1, FAR const struct uint64_s *factor2,
              FAR struct uint64_s *product);

/****************************************************************************
 * Name: umul64
 *
 * Description:
 *   Multiply two 64-bit values, factor1 and factor2, and return the
 *   truncated 64-bit product.
 *
 * Input Parameters:
 *   factor1 and factor2 - The values to be multiplied
 *   product - The location to return the product of the two values.
 *
 ****************************************************************************/

void umul64(FAR const struct uint64_s *factor1,
            FAR const struct uint64_s *factor2,
            FAR struct uint64_s *product);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_MATH32_H */
