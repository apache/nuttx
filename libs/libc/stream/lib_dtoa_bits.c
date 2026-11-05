/****************************************************************************
 * libs/libc/stream/lib_dtoa_bits.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <string.h>

#include "lib_dtoa_engine.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The format of a double.  There is no floating point operation here, so a
 * CPU with no FPU needs no soft-float code.
 */

#if DBL_MANT_DIG == 53
typedef uint64_t dtoa_bits_t;
#  define DTOA_EXP_MASK    0x7ff
#  define DTOA_EXP_BIAS    1023
#  define DTOA_FRAC_BITS   52
#  define DTOA_MANT_SHIFT  11
#  define DTOA_POW_NUM     9
#elif DBL_MANT_DIG == 24
typedef uint32_t dtoa_bits_t;
#  define DTOA_EXP_MASK    0xff
#  define DTOA_EXP_BIAS    127
#  define DTOA_FRAC_BITS   23
#  define DTOA_MANT_SHIFT  40
#  define DTOA_POW_NUM     6
#else
#  error "The format of double is not known"
#endif

#define DTOA_FIX_BITS      60
#define DTOA_FIX_MASK      ((1ull << DTOA_FIX_BITS) - 1)

/* No value has more decimals than this, so a larger one means the same */

#define DTOA_MAX_DECIMALS  20000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Where the digits go when they come from the first one to the last one */

struct dtoa_out_s
{
  FAR char *digits;
  int32_t exp;
  int32_t limit;
  int32_t n;
  int8_t next;
  bool started;
  bool rest;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 10^(2^i) and 10^-(2^i): the mantissa with its top bit set, and b, so that
 * the value is m * 2^(b - 63).
 */

static const uint64_t g_dtoa_up[DTOA_POW_NUM] =
{
  0xa000000000000000ull,
  0xc800000000000000ull,
  0x9c40000000000000ull,
  0xbebc200000000000ull,
  0x8e1bc9bf04000000ull,
  0x9dc5ada82b70b59eull,
#if DTOA_POW_NUM > 6
  0xc2781f49ffcfa6d5ull,
  0x93ba47c980e98ce0ull,
  0xaa7eebfb9df9de8eull,
#endif
};

static const uint64_t g_dtoa_down[DTOA_POW_NUM] =
{
  0xcccccccccccccccdull,
  0xa3d70a3d70a3d70aull,
  0xd1b71758e219652cull,
  0xabcc77118461cefdull,
  0xe69594bec44de15bull,
  0xcfb11ead453994baull,
#if DTOA_POW_NUM > 6
  0xa87fea27a539e9a5ull,
  0xddd0467c64bce4a1ull,
  0xc0314325637a193aull,
#endif
};

static const int16_t g_dtoa_up_b[DTOA_POW_NUM] =
{
  3, 6, 13, 26, 53, 106,
#if DTOA_POW_NUM > 6
  212, 425, 850,
#endif
};

static const int16_t g_dtoa_down_b[DTOA_POW_NUM] =
{
  -4, -7, -14, -27, -54, -107,
#if DTOA_POW_NUM > 6
  -213, -426, -851,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dtoa_zero(FAR struct dtoa_s *dtoa)
{
  dtoa->digits[0] = '0';
  dtoa->digits[1] = '\0';
  dtoa->exp = 0;
}

/* Round to nearest, ties to even.  next is the first digit that was cut, and
 * rest says if a digit after it is not 0.
 */

static int dtoa_finish(FAR struct dtoa_s *dtoa, int32_t n, int32_t exp,
                       int next, bool rest, int32_t limit, bool fixed)
{
  int32_t i;

  if (next > 5 || (next == 5 && (rest ||
      (n > 0 && ((dtoa->digits[n - 1] - '0') & 1) != 0))))
    {
      for (i = n - 1; i >= 0; i--)
        {
          if (dtoa->digits[i] != '9')
            {
              dtoa->digits[i]++;
              break;
            }

          dtoa->digits[i] = '0';
        }

      if (i < 0)
        {
          /* 999 becomes 1000, or no digits become 1 */

          dtoa->digits[0] = '1';
          exp++;
          if (n == 0 || (fixed && n < limit))
            {
              if (n > 0)
                {
                  dtoa->digits[n] = '0';
                }

              n++;
            }
        }
    }
  else if (n == 0)
    {
      dtoa_zero(dtoa);
      return 1;
    }

  if (!fixed)
    {
      while (n < limit)
        {
          dtoa->digits[n++] = '0';
        }
    }

  dtoa->digits[n] = '\0';
  dtoa->exp = exp;
  return n;
}

static void dtoa_put(FAR struct dtoa_out_s *o, int d, int32_t decimals)
{
  if (!o->started)
    {
      if (d == 0)
        {
          o->exp--;
          return;
        }

      o->started = true;
      if (decimals >= 0)
        {
          int32_t lim = decimals + o->exp + 1;

          o->limit = lim < 0 ? -1 : lim < o->limit ? lim : o->limit;
        }
    }

  if (o->n < o->limit)
    {
      o->digits[o->n++] = d + '0';
    }
  else if (o->next < 0)
    {
      o->next = d;
    }
  else if (d != 0)
    {
      o->rest = true;
    }
}

/* An exact path for a value with a short integer and a short fraction.  It
 * has the ties of everyday values right: 2.5, 99.5, 1.125.  The value is
 * m * 2^e2, below 2^63, with at most 60 bits after the point.
 */

static int dtoa_fixed(uint64_t m, int e2, FAR struct dtoa_s *dtoa,
                      int32_t max_digits, int32_t decimals)
{
  struct dtoa_out_s o;
  uint64_t ipart = m;
  uint64_t frac = 0;
  char tmp[20];
  int len = 0;
  int i;

  if (e2 >= 0)
    {
      for (i = 0; i < e2; i++)
        {
          ipart <<= 1;
        }
    }
  else
    {
      frac = m;
      for (i = -e2; i < DTOA_FIX_BITS; i++)
        {
          frac <<= 1;
        }

      frac &= DTOA_FIX_MASK;

      for (i = 0; i < -e2 && ipart != 0; i++)
        {
          ipart >>= 1;
        }
    }

  while (ipart != 0)
    {
      tmp[len++] = ipart % 10;
      ipart /= 10;
    }

  o.digits  = dtoa->digits;
  o.exp     = len > 0 ? len - 1 : -1;
  o.limit   = max_digits;
  o.n       = 0;
  o.next    = -1;
  o.started = false;
  o.rest    = false;

  while (len > 0)
    {
      dtoa_put(&o, tmp[--len], decimals);
    }

  while (frac != 0 && !o.rest && !(o.started && o.limit < 0))
    {
      frac *= 10;
      dtoa_put(&o, frac >> DTOA_FIX_BITS, decimals);
      frac &= DTOA_FIX_MASK;
    }

  if (o.limit < 0)
    {
      dtoa_zero(dtoa);
      return 1;
    }

  return dtoa_finish(dtoa, o.n, o.exp, o.next, o.rest, max_digits,
                     decimals >= 0);
}

/* The top half of a 64 x 64 bit product, rounded on the first bit it drops */

static uint64_t dtoa_mulhi(uint64_t a, uint64_t b)
{
  uint32_t a0 = a;
  uint32_t a1 = a >> 32;
  uint32_t b0 = b;
  uint32_t b1 = b >> 32;
  uint64_t p00 = (uint64_t)a0 * b0;
  uint64_t p01 = (uint64_t)a0 * b1;
  uint64_t p10 = (uint64_t)a1 * b0;
  uint64_t p11 = (uint64_t)a1 * b1;
  uint64_t mid = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;

  return p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32) +
         ((mid >> 31) & 1);
}

/* m has its top bit set, and the value is m * 2^(b - 63).  Scale it by a
 * power of ten to 1 <= y < 10, in 64 bits with 4 bits before the point.
 */

static int dtoa_scaled(uint64_t m, int b, FAR struct dtoa_s *dtoa,
                       int32_t max_digits, int32_t decimals)
{
  uint64_t y;
  int32_t exp;
  int32_t n;
  int32_t i;
  int k;

  k = (int)(((int32_t)(b + 1) * 78913) >> 18);
  exp = k;

  if (k != 0)
    {
      unsigned int bit = k < 0 ? -k : k;

      for (i = 0; i < DTOA_POW_NUM && bit != 0; i++, bit >>= 1)
        {
          if ((bit & 1) != 0)
            {
              m = dtoa_mulhi(m, k < 0 ? g_dtoa_up[i] : g_dtoa_down[i]);
              b += (k < 0 ? g_dtoa_up_b[i] : g_dtoa_down_b[i]) + 1;
              if ((m >> 63) == 0)
                {
                  m <<= 1;
                  b--;
                }
            }
        }
    }

  /* The estimate gives 0.5 <= y < 10, and rarely a little more */

  if (b < 0)
    {
      m = dtoa_mulhi(m, g_dtoa_up[0]);
      b += g_dtoa_up_b[0] + 1;
      if ((m >> 63) == 0)
        {
          m <<= 1;
          b--;
        }

      exp--;
    }
  else if (b > 3)
    {
      m = dtoa_mulhi(m, g_dtoa_down[0]);
      b += g_dtoa_down_b[0] + 1;
      if ((m >> 63) == 0)
        {
          m <<= 1;
          b--;
        }

      exp++;
    }

  y = m;
  for (i = b; i < 3; i++)
    {
      y >>= 1;
    }

  n = max_digits;
  if (decimals >= 0)
    {
      int32_t lim = decimals + exp + 1;

      if (lim < 0)
        {
          dtoa_zero(dtoa);
          return 1;
        }

      if (lim < n)
        {
          n = lim;
        }
    }

  for (i = 0; i < n; i++)
    {
      dtoa->digits[i] = (char)(y >> DTOA_FIX_BITS) + '0';
      y = (y & DTOA_FIX_MASK) * 10;
    }

  return dtoa_finish(dtoa, n, exp, (int)(y >> DTOA_FIX_BITS),
                     (y & DTOA_FIX_MASK) != 0, max_digits, decimals >= 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __dtoa_engine
 *
 * Description:
 *   Give the first digits of x, rounded.  max_digits is the count of digits,
 *   and with max_decimals of 0 or more the digits end at that decimal.  At
 *   most DTOA_MAX_DIG digits come back.
 *
 * Returned Value:
 *   The number of digits in dtoa->digits, with dtoa->exp the exponent of the
 *   first one.
 *
 ****************************************************************************/

int __dtoa_engine(double x, FAR struct dtoa_s *dtoa, int max_digits,
                  int max_decimals)
{
  dtoa_bits_t u;
  int32_t decimals = max_decimals;
  int32_t limit = max_digits;
  uint64_t m;
  uint64_t n;
  int ef;
  int e2;
  int b;
  int i;

  memcpy(&u, &x, sizeof(u));
  ef = (u >> DTOA_FRAC_BITS) & DTOA_EXP_MASK;
  m = u & ((1ull << DTOA_FRAC_BITS) - 1);

  if (limit > DTOA_MAX_DIG || decimals >= 0)
    {
      limit = DTOA_MAX_DIG;
    }

  if (decimals > DTOA_MAX_DECIMALS)
    {
      decimals = DTOA_MAX_DECIMALS;
    }

  dtoa->flags = (u >> (sizeof(u) * 8 - 1)) != 0 ? DTOA_MINUS : 0;
  dtoa->exp = 0;

  if (ef == DTOA_EXP_MASK)
    {
      dtoa->flags |= m != 0 ? DTOA_NAN : DTOA_INF;
      dtoa->digits[0] = '\0';
      return 0;
    }

  if (ef == 0 && m == 0)
    {
      dtoa->flags |= DTOA_ZERO;
      if (decimals >= 0)
        {
          limit = 1;
        }

      for (i = 0; i < limit; i++)
        {
          dtoa->digits[i] = '0';
        }

      dtoa->digits[limit] = '\0';
      return limit;
    }

  /* The value is m * 2^e2 */

  if (ef != 0)
    {
      m |= 1ull << DTOA_FRAC_BITS;
      e2 = ef - DTOA_EXP_BIAS - DTOA_FRAC_BITS;
    }
  else
    {
      e2 = 1 - DTOA_EXP_BIAS - DTOA_FRAC_BITS;
    }

  n = m << DTOA_MANT_SHIFT;
  b = e2 - DTOA_MANT_SHIFT + 63;
  while ((n >> 63) == 0)
    {
      n <<= 1;
      b--;
    }

  if (b < 63 && e2 >= -DTOA_FIX_BITS)
    {
      return dtoa_fixed(m, e2, dtoa, limit, decimals);
    }

  return dtoa_scaled(n, b, dtoa, limit, decimals);
}
