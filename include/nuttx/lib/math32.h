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
 * or if n is zero, returns zero.  The argument is evaluated multiple
 * times and must be free of side effects.
 */

#define FLS32(n) ((n) & 0xffff0000 ? 16 + FLS16((n) >> 16) : FLS16(n))
#define FLS16(n) ((n) & 0xff00     ?  8 + FLS8 ((n) >>  8) : FLS8 (n))
#define FLS8(n)  ((n) & 0xf0       ?  4 + FLS4 ((n) >>  4) : FLS4 (n))
#define FLS4(n)  ((n) & 0xc        ?  2 + FLS2 ((n) >>  2) : FLS2 (n))
#define FLS2(n)  ((n) & 0x2        ?  1 + FLS1 ((n) >>  1) : FLS1 (n))
#define FLS1(n)  ((n) & 0x1        ?  1 : 0)

/* Widen before shifting so narrow types never see oversized shift counts. */

#define FLS64(n) ((uint64_t)(n) >> 32 ? \
                  32 + FLS32((uint32_t)((uint64_t)(n) >> 32)) : FLS32(n))

#ifdef __INT128_DEFINED
#  define FLS128(n) ((_uint128_t)(n) >> 64 ? \
                     64 + FLS64((uint64_t)((_uint128_t)(n) >> 64)) : FLS64(n))
#  define FLS(n) (sizeof((n)) <= sizeof(uint64_t) ? FLS64(n) : FLS128(n))
#else
#  define FLS(n) FLS64(n)
#endif

/* Checks if an integer is power of two at compile time */

#define IS_POWER_OF_2(n)           ((n) > 0 && ((n) & (n - 1)) == 0)

/* Returns the log2(n) rounded up and down; usable at compile time.
 * n >= 1 is required: log2 is undefined for 0 and negative arguments.
 * As with FLS(), n is evaluated multiple times.
 */

#define LOG2_FLOOR(n) (FLS(n) - 1)

#ifdef CONFIG_HAVE_BUILTIN_CLZ
#  define LOG2_CEIL(n) log2ceil(n)
#else
#  define LOG2_CEIL(n) (FLS((n) - 1))
#endif

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
 *   Calculate the base-2 logarithm of n, rounded up.
 *
 * Input Parameters:
 *   n - The value to take the logarithm of (n >= 1).
 *
 * Returned Value:
 *   ceil(log2(n)).
 *
 ****************************************************************************/

/* ceil(log2 n) is fls(n - 1).  fls() of 0 -- the n == 1 case -- is
 * undefined under __builtin_clz(), so OR the zero away and subtract the
 * n == 1 term: branch-free, and still an integer constant expression.
 */

#define log2ceil(n) (flsx(((n) - 1) | 1) - ((n) == 1))

/****************************************************************************
 * Name: log2floor
 *
 * Description:
 *   Calculate the base-2 logarithm of n, rounded down.
 *
 * Input Parameters:
 *   n - The value to take the logarithm of (n >= 1).
 *
 * Returned Value:
 *   floor(log2(n)).
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
 * For n/d where d is an invariant integer, fill the parameter block
 * with invdiv_init_param(d, &ip), then divide with invdiv_u32() or
 * invdiv_u64() to match the block width.
 */

/****************************************************************************
 * Name: invdiv_init_param
 *
 * Description:
 *   Fill an invdiv_param32_t or an invdiv_param64_t block with the
 *   invariant-division parameters of d; the width of ip->mult selects
 *   the initializer.  d may be a constant or a run-time value.
 *
 * Input Parameters:
 *   d  - The divisor, d > 1.
 *   ip - The parameter block; its type selects the initializer.
 *
 ****************************************************************************/

/* Dispatch on the multiplier width of the parameter block: d may be a
 * runtime value or a constant, either way l is computed inside.
 */

#define invdiv_init_param(d, ip) \
  sizeof((ip)->mult) == sizeof(uint32_t) ? \
    invdiv_init_param32((d), (FAR invdiv_param32_t *)(ip)) : \
    invdiv_init_param64((d), (FAR invdiv_param64_t *)(ip))

/****************************************************************************
 * Name: invdiv_init_param32
 *
 * Description:
 *   Calculate the invariant-division parameters (multiplier, shift) for
 * divisor d during initialization.
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
  int      l  = LOG2_CEIL(d);
  uint64_t t1 = (uint64_t)1u << 32;
  uint64_t t2 = (uint64_t)1u << l;

  DEBUGASSERT(d > 1u);

  ip->mult  = (t1 * (t2 - d)) / d + 1u;
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
 *   where d is already converted to invdiv_param32_t.
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

/* One Knuth-D correction step: while qv*v0v exceeds rv*2^32 + lx, step
 * qv back one and return v1v to rv.  A normalized divisor bounds it to
 * two per digit, one when qv was clamped to b-1; the rv < 2^32 guard is
 * exact.  The arguments are evaluated more than once.
 */

#define KD2_CORR(qv, rv, v1v, v0v, lx) \
do \
  { \
    uint64_t c_ = ((rv) < (1ULL << 32)) && \
                  ((qv) * (v0v) > (((rv) << 32) | (lx))); \
    (qv) -= c_; \
    (rv) += c_ * (v1v); \
  } \
while (0)

/****************************************************************************
 * Name: invdiv_udiv128_soft
 *
 * Description:
 *   Divide (hi*2^64 + lo) by d for hi < d, where the quotient fits 64
 *   bits and is returned whole.  s = clz(d) is supplied by the caller.
 *
 * Input Parameters:
 *   hi - The high 64 bits of the dividend.
 *   lo - The low 64 bits of the dividend.
 *   d  - The divisor.
 *   s  - The normalizer, clz(d).
 *
 * Returned Value:
 *   floor((hi*2^64 + lo) / d); hi < d is a precondition.
 *
 ****************************************************************************/

static inline_function uint64_t
invdiv_udiv128_soft(uint64_t hi, uint64_t lo, uint64_t d, int s)
{
  /* Hand long division in base b = 2^32 (Knuth, TAOCP vol. 2, 4.3.1,
   * algorithm D):
   *
   *   D    = d<<s            = v1*b + v0
   *   N<<s = nh*b^2 + l1*b + l0
   *   Q    = floor(N / d)    = q1*b + q0
   *
   * One quotient digit, with u the next dividend digit:
   *
   *   q = min(floor(nh / v1), b-1)     trial quotient
   *   r = nh - q*v1
   *   while (q*v0 > r*b + u)           normalized d: <= 2 times
   *     { q--;  r += v1; }
   *   nh = r*b + u - q*v0              subtract, bring down
   *
   * KD2_CORR() is one while-step; three slots per digit hold the
   * theorem-B bound without a branch.  Round 1 takes u = l1, round 2
   * u = l0; the digits assemble Q = q1*b + q0.  Only Q is wanted.
   */

  uint64_t dv = d << s;
  uint64_t v1 = dv >> 32;
  uint64_t v0 = dv & UINT32_MAX;

  /* s == 0 would make lo >> (64 - s) a 64-bit shift (UB). */

  uint64_t nh = s ? ((hi << s) | (lo >> (64 - s))) : hi;
  uint64_t nl = lo << s;
  uint64_t l1 = nl >> 32;
  uint64_t l0 = nl & UINT32_MAX;
  uint64_t q1;
  uint64_t r1;
  uint64_t R;
  uint64_t q0;
  uint64_t r0;

  /* digit 1 */

  q1 = nh / v1;
  q1 = q1 > UINT32_MAX ? UINT32_MAX : q1;
  r1 = nh - q1 * v1;
  KD2_CORR(q1, r1, v1, v0, l1);
  KD2_CORR(q1, r1, v1, v0, l1);
  KD2_CORR(q1, r1, v1, v0, l1);

  R  = ((r1 << 32) | l1) - q1 * v0;

  /* digit 2 */

  q0 = R / v1;
  q0 = q0 > UINT32_MAX ? UINT32_MAX : q0;
  r0 = R - q0 * v1;
  KD2_CORR(q0, r0, v1, v0, l0);
  KD2_CORR(q0, r0, v1, v0, l0);
  KD2_CORR(q0, r0, v1, v0, l0);

  return (q1 << 32) + q0;
}

/* UMULH: high 64 bits of a * b.  One instruction inline when the
 * compiler has a native 128-bit multiply; otherwise one out-of-line copy
 * per image from libs/libc/machine (the architecture form, or the weak
 * fallback).
 */

#ifdef __INT128_DEFINED

static inline_function uint64_t up_umulh64(uint64_t a, uint64_t b)
{
  /* Compiles to a single UMULH instruction on x86_64 and AArch64. */

  _uint128_t a128 = a;
  _uint128_t res128 = a128 * b;
  return res128 >> 64;
}

#else
uint64_t up_umulh64(uint64_t a, uint64_t b);
#endif

/****************************************************************************
 * Name: invdiv_init_param64
 *
 * Description:
 *   Calculate the invariant-division parameters from the divisor, with
 *   l = ceil(log2 d) and the multiplier floor((2^l - d) * 2^64 / d) + 1
 *   from one 128-by-64 division.  The normalizer s = clz(d) is derived
 *   from l without a branch.
 *
 * Input Parameters:
 *   d - The divisor (unsigned integer), d != 0 and d != 1 is required,
 *       and d <= 2^63 is asserted: the 1 << l setup is undefined for
 *       larger divisors.
 *
 * Output Parameters:
 *   ip    - The invariant division parameter to be cached.
 *
 ****************************************************************************/

static inline_function
void invdiv_init_param64(uint64_t d, FAR invdiv_param64_t *ip)
{
  int      l = LOG2_CEIL(d);
  uint64_t t = ((uint64_t)1u << l) - d;

  DEBUGASSERT(d > 1u && d <= ((uint64_t)1u << 63));

#ifdef __INT128_DEFINED
  _uint128_t one128 = 1u;

  ip->mult = (one128 << 64) * t / d + 1u;
#else
  int s = 64 - l - (((d - 1u) & d) == 0u);  /* = clz(d) */

  ip->mult = invdiv_udiv128_soft(t, 0, d, s) + 1u;
#endif
  ip->shift = l - 1;
}

/****************************************************************************
 * Name: invdiv_u64
 *
 * Description:
 *   Return the result of `n / d`
 *   The division is realized by multiplication and right shift,
 *   where d is already converted to invdiv_param64_t.
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
  uint8_t  sh1 = 1u;
  uint8_t  sh2 = ip->shift;

  /* t1 is the high half of the 128-bit product n * m.  Both operands
   * are runtime values here, so up_umulh64() is the right primitive.
   */

  t1 = up_umulh64(n, m);
  t2 = (n - t1) >> sh1;

  return (t1 + t2) >> sh2;
}

static inline_function uint64_t div64_const32(uint64_t n, uint32_t b)
{
  invdiv_param64_t ip;

  DEBUGASSERT(b > 1u);
  invdiv_init_param(b, &ip);

  return invdiv_u64(n, &ip);
}

#ifdef CONFIG_DEBUG_NOOPT

/* Without optimization the magic setup is not folded and runs on every
 * call, far slower than a plain division.  The cast is on the result,
 * not the operands: div_const() returns uint64_t in every build while
 * the division keeps the operands' native width.
 */

#  define div64_const(n, base) ((uint64_t)((n) / (base)))
#else

/* The divisor is expected to be a compile-time constant so the magic
 * setup below folds away; for runtime-invariant divisors use the invdiv
 * API instead.
 */

static inline_function uint64_t div64_const(uint64_t n, uint64_t base)
{
  DEBUGASSERT(base > 0u);

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
