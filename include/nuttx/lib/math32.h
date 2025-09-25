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
#include <nuttx/compiler.h>

#include <assert.h>
#include <limits.h>
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

typedef struct invdiv_param32_s
{
  uint32_t mult;
  uint8_t  shift;
} invdiv_param32_t;

typedef struct invdiv_param64_s
{
  uint64_t mult;
  uint8_t  shift;
} invdiv_param64_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
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

/* Default C implementation for umul64_const()
 *
 * Prototype: uint64_t umul64_const(uint64_t m, uint64_t n, int bias);
 * Semantic:  return ((bias ? m : 0) + m * n) >> 64
 *
 * The product is a 128-bit value, scaled down to 64 bits.
 * Assuming constant propagation to optimize away unused conditional code.
 * Architectures may provide their own optimized assembly implementation.
 */

/* Due to compilation issues, use inline functions instead of macros. */

#ifdef up_umul64_const
#  define umul64_const(m, n, bias) up_umul64_const(m, n, bias)
#else
static inline_function uint64_t umul64_const(uint64_t m, uint64_t n,
                                             int bias)
{
  uint32_t m_lo = (m) & 0xffffffff;
  uint32_t m_hi = (m) >> 32;
  uint32_t n_lo = (n) & 0xffffffff;
  uint32_t n_hi = (n) >> 32;
  uint32_t res_lo;
  uint32_t res_hi;
  uint32_t tmp;
  uint64_t res;

  if (!(bias))
    {
      res = ((uint64_t)m_lo * n_lo) >> 32;
    }
  else if (!((m) & ((1ULL << 63) | (1ULL << 31))))
    {
      res = ((m) + (uint64_t)m_lo * n_lo) >> 32;
    }
  else
    {
      res = (m) + (uint64_t)m_lo * n_lo;
      res_lo = res >> 32;
      res_hi = (res_lo < m_hi);
      res = res_lo | ((uint64_t)res_hi << 32);
    }

  if (!((m) & ((1ULL << 63) | (1ULL << 31))))
    {
      res += (uint64_t)m_lo * n_hi;
      res += (uint64_t)m_hi * n_lo;
      res >>= 32;
    }
  else
    {
      res += (uint64_t)m_lo * n_hi;
      tmp = res >> 32;
      res += (uint64_t)m_hi * n_lo;
      res_lo = res >> 32;
      res_hi = (res_lo < tmp);
      res = res_lo | ((uint64_t)res_hi << 32);
    }

  res += (uint64_t)m_hi * n_hi;

  return res;
}
#endif

static inline_function uint64_t div64_const32(uint64_t n, uint32_t b)
{
  uint64_t res;
  uint64_t x;
  uint64_t t;
  uint64_t m;
  uint32_t p;
  uint32_t bias;

  DEBUGASSERT(b > 1);

  p = 1u << LOG2_FLOOR(b);
  m = (~0ULL / b) * p;
  m += (((~0ULL % b + 1u) * p) + b - 1u) / b;
  x = ~0ULL / b * b - 1u;
  res = ((m & 0xffffffffu) * (x & 0xffffffffu)) >> 32;
  t = res += (m & 0xffffffffu) * (x >> 32);
  res += (x & 0xffffffffu) * (m >> 32);
  t = (res < t) ? (1ULL << 32) : 0u;
  res = (res >> 32) + t;
  res += (m >> 32) * (x >> 32);
  res /= p;

  if (~0ULL % (b / (b & -b)) == 0u)
    {
      n /= (b & -b);
      m = ~0ULL / (b / (b & -b));
      p = 1u;
      bias = 1u;
    }
  else if (res != x / b)
    {
      bias = 1u;
      m = (~0ULL / b) * p;
      m += ((~0ULL % b + 1u) * p) / b;
    }
  else
    {
      uint32_t bits = -(m & -m);
      bits |= m >> 32;
      bits = (~bits) << 1;
      if (!bits)
        {
          p /= (m & -m);
          m /= (m & -m);
        }
      else
        {
          p >>= LOG2_FLOOR(bits);
          m >>= LOG2_FLOOR(bits);
        }

      bias = 0u;
    }

  res = umul64_const(m, n, bias);
  res /= p;

  return res;
}

#ifdef CONFIG_DEBUG_NOOPT

/* Without optimization the magic number setup is not folded at compile
 * time but runs on every call (it even contains several divisions),
 * which is far slower than a plain division.  Cast the result, not the
 * operands: div_const() must return uint64_t in every build, but the
 * division itself keeps the operands' native width.
 */

#  define div64_const(n, base) ((uint64_t)((n) / (base)))
#else

/* The divisor is expected to be a compile-time constant so that the
 * magic multiplier setup below is folded away; for runtime-invariant
 * divisors use the invdiv API instead.
 */

static inline_function uint64_t div64_const(uint64_t n, uint64_t base)
{
  DEBUGASSERT(base > 0);

  if (IS_POWER_OF_2(base))
    {
      n >>= LOG2_FLOOR(base);
    }
  else if (base <= UINT32_MAX)
    {
      n = div64_const32(n, (uint32_t)base);
    }
  else
    {
      n /= base;
    }

  return n;
}
#endif

/* The div_const*() family takes non-negative arguments only,
 * negative values are outside the contract.
 */

#define div_const(n, base) \
  ((is_constexpr(base) && !is_constexpr(n) && \
    sizeof((n)) == sizeof(uint64_t) && UINTPTR_MAX == UINT32_MAX) ? \
   div64_const(n, base) : ((n) / (base)))

/* Unit constant in the essential type of x (x > 0), so MISRA C Rule 10.4
 * holds for both signed and unsigned arguments.
 */

#define MATH32_ONE(x) ((x) / (x))

#define div_const_roundup(n, base) \
  div_const((n) + (base) - MATH32_ONE(base), base)
#define div_const_roundnearest(n, base) \
  div_const((n) + ((base) / (MATH32_ONE(base) + MATH32_ONE(base))), base)

/* Division optimizing method proposed by T. Granlund and L. Montgomery.
 * This method converts the runtime-invariant integer division
 * into multiplication and right-shifting.
 *
 * Usage:
 * If you want to do n/d division where d is an invariant integer,
 * initialize the param by `invdiv_init_param(d, param)` first,
 * then do the division by `invdiv(n, param)`.
 */

/****************************************************************************
 * Name: invdiv_init_param32
 *
 * Description:
 *   Calculate the triple (multiplier, right shift number 1, right shift
 * number 2) during initialization.
 *
 * Input Parameters:
 *   d - The divisor (unsigned integer). d != 0 and d != 1 is required.
 *
 * Output Parameters:
 *   ip    - The invariant division parameter to be cached.
 *
 ****************************************************************************/

static inline_function
void invdiv_init_param32(uint32_t d, FAR invdiv_param32_t *ip)
{
  int      l  = log2ceil(d);
  uint64_t t1 = (uint64_t)1 << 32;
  uint64_t t2 = (uint64_t)1 << l;

  DEBUGASSERT(d > 1);

  ip->mult  = (t1 * (t2 - d)) / d + 1;
#if ULONG_MAX == 4294967295UL
  ip->shift = l - 1;
#else
  ip->shift = l;
#endif
}

/****************************************************************************
 * Name: invdiv_u32
 *
 * Description:
 *   Return the result of `n / d`
 *   The division is realized by multiplication and right shift,
 *   where d is already converted to invdiv_param_t.
 *
 * Input Parameters:
 *   n - The dividend (uint32_t).
 *   ip    - The invariant division parameter already cached.
 *
 * Returned Value:
 *  The result of `n / d`
 *
 ****************************************************************************/

static inline_function
uint64_t invdiv_u32(uint32_t n, FAR const invdiv_param32_t *ip)
{
  uint8_t  sh = ip->shift;
#if ULONG_MAX == 4294967295UL
  uint32_t m  = ip->mult;
  uint32_t t1 = ((uint64_t)m * n) >> 32; /* UMULH if supported */
  uint32_t t2 = (n - t1) >> 1;
#else
  /* This division can be 25% faster using 64-bit register. */

  uint64_t m  = ip->mult;
  uint64_t t1 = (m * n) >> 32;
  uint32_t t2 = n;
#endif

  return (t1 + t2) >> sh;
}

/* Helper function to do n bits integer division. */

#define invdiv_udiv_soft(arr, n, q, d) \
do \
{ \
    uint32_t idx; \
    uint32_t bits_per_idx = sizeof(uint64_t) * 8; \
    uint32_t bits_max = bits_per_idx * n; \
    uint64_t reminder = 0ull; \
    uint64_t high_rem = 0ull; \
    for (idx = 0; idx < bits_max; idx++) \
      { \
        uint32_t bits = bits_max - idx - 1; \
        uint32_t bits_idx = bits / bits_per_idx; \
        uint32_t bits_idx_off = bits % bits_per_idx; \
        reminder <<= 1u; \
        reminder |= ((arr)[bits_idx] >> bits_idx_off) & 1u; \
        if (reminder >= (d)) \
          { \
            reminder    -= (d); \
            q[bits_idx] |= (1ull << bits_idx_off); \
          } \
        else if (high_rem != 0) \
          { \
            uint64_t c   = 0ull - (d); \
            high_rem    -= 1; \
            reminder    += c; \
            q[bits_idx] |= (1ull << bits_idx_off); \
          } \
        high_rem += (reminder >> (bits_per_idx - 1)) & 1u; \
      } \
} \
while(0)

/* UMULH: high 64 bits of a * b.  Architectures may override it with a
 * native or assembly implementation via up_umulh64(), which must
 * evaluate both arguments.
 */

#ifdef up_umulh64
#  define umulh64(a, b) up_umulh64(a, b)
#else

static inline_function uint64_t umulh64(uint64_t a, uint64_t b)
{
#ifndef __INT128_DEFINED
  /* If the compiler do not support _uint128_t */

  uint64_t al = a & UINT32_MAX;
  uint64_t ah = a >> 32;
  uint64_t bl = b & UINT32_MAX;
  uint64_t bh = b >> 32;

  uint64_t m1 = al * bl;
  uint64_t m2 = ah * bl;
  uint64_t m3 = al * bh;
  uint64_t m4 = ah * bh;

  uint64_t c = (m1 >> 32) + (m2 & UINT32_MAX) + (m3 & UINT32_MAX);

  return m4 + (m2 >> 32) + (m3 >> 32) + (c >> 32);
#else
  /* Compiles to a single UMULH instruction on x86_64 and AArch64. */

  _uint128_t a128 = a;
  _uint128_t res128 = a128 * b;
  return res128 >> 64;
#endif
}
#endif

/****************************************************************************
 * Name: invdiv_init_param64
 *
 * Description:
 *   Calculate the triple (multiplier, right shift number 1, right shift
 * number 2) during initialization.
 *
 * Input Parameters:
 *   d - The divisor (unsigned integer), d != 0 and d != 1 is required.
 *
 * Output Parameters:
 *   ip    - The invariant division parameter to be cached.
 *
 ****************************************************************************/

static inline_function
void invdiv_init_param64(uint64_t d, FAR invdiv_param64_t *ip)
{
  int      l = log2ceil(d);
  uint64_t t = ((uint64_t)1 << l) - d;

  DEBUGASSERT(d > 1);

#ifndef __INT128_DEFINED
  /* If the compiler do not support UINT128 */

  uint64_t q[2];
  uint64_t mres[2];

  /* Calculate the mult via 2^64 * (2^l - d) / d + 1
   * It is equal to ((2^l - d) << 64) / d + 1. So...
   */

  mres[1] = t;
  mres[0] = 0;
  q[0] = 0;
  q[1] = 0;

  /* Then, do the 128-bit division. */

  invdiv_udiv_soft(mres, 2, q, d);

  ip->mult = q[0] + 1;
#else
  _uint128_t one128 = 1;
  ip->mult = (one128 << 64) * t / d + 1;
#endif
  ip->shift = l - 1;
}

/****************************************************************************
 * Name: invdiv_u64
 *
 * Description:
 *   Return the result of `n / d`
 *   The division is realized by multiplication and right shift,
 *   where d is already converted to invdiv_param_t.
 *
 * Input Parameters:
 *   n - The dividend (uint64_t).
 *   ip    - The invariant division parameter already cached.
 *
 * Returned Value:
 *  The result of `n / d`
 *
 ****************************************************************************/

static inline_function
uint64_t invdiv_u64(uint64_t n, FAR const invdiv_param64_t *ip)
{
  uint64_t t1;
  uint64_t t2;
  uint64_t m   = ip->mult;
  uint8_t  sh1 = 1;
  uint8_t  sh2 = ip->shift;

  /* t1 is the high half of the 128-bit product n * m.  Both operands
   * are runtime values here, so umulh64() is the right primitive.
   */

  t1 = umulh64(n, m);
  t2 = (n - t1) >> sh1;

  return (t1 + t2) >> sh2;
}

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

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LIB_MATH32_H */
