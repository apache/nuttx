/****************************************************************************
 * libs/libc/stream/lib_dtoa_big.c
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

#ifdef CONFIG_ENDIAN_BIG
#  define DTOA_W_HI 0
#  define DTOA_W_LO 1
#else
#  define DTOA_W_HI 1
#  define DTOA_W_LO 0
#endif

/* An up value: no digit grows, or the digits were all 9 */

#define DTOA_UP_NONE  -1
#define DTOA_UP_NINES -2

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* d = d * mult + add, in base 10000 */

static int big_mul(FAR uint16_t *d, int count, uint32_t mult, uint16_t add)
{
  uint32_t carry = add;
  int i;

  for (i = 0; i < count; i++)
    {
      uint32_t t = d[i] * mult + carry;

      d[i] = t % 10000;
      carry = t / 10000;
    }

  while (carry != 0)
    {
      d[count++] = carry % 10000;
      carry /= 10000;
    }

  return count;
}

/* Start at the first digit: the integer part, as limbs in base 10000 */

static void big_restart(FAR struct dtoa_big_s *b)
{
  uint64_t hi = b->hi;
  uint64_t lo = b->lo;
  int32_t fb = b->e2 < 0 ? -b->e2 : 0;
  int32_t i;
  int count = 0;

  for (i = 0; i < fb && (hi | lo) != 0; i++)
    {
      lo = (lo >> 1) | (hi << 63);
      hi >>= 1;
    }

  if ((hi | lo) != 0)
    {
      for (i = 0; i < 4; i++)
        {
          count = big_mul(b->limbs, count, 65536, (hi >> 48) & 0xffff);
          hi <<= 16;
        }

      for (i = 0; i < 4; i++)
        {
          count = big_mul(b->limbs, count, 65536, (lo >> 48) & 0xffff);
          lo <<= 16;
        }

      for (i = b->e2; i > 0; i -= 13)
        {
          count = big_mul(b->limbs, count, 1u << (i > 13 ? 13 : i), 0);
        }
    }

  b->limb = count;
  b->ingroup = 0;
  b->infrac = false;
  b->nl = 0;
  b->pos = 0;
}

/* The fraction as limbs in base 65536, with the top one first */

static void big_loadfrac(FAR struct dtoa_big_s *b)
{
  uint64_t hi = b->hi;
  uint64_t lo = b->lo;
  int32_t fb = b->e2 < 0 ? -b->e2 : 0;
  int32_t i;

  b->infrac = true;
  b->nl = (fb + 15) / 16;
  memset(b->limbs, 0, b->nl * sizeof(uint16_t));

  for (i = 0; i < fb && (hi | lo) != 0; i++)
    {
      if ((lo & 1) != 0)
        {
          int32_t t = fb - i - 1;

          b->limbs[t >> 4] |= 0x8000u >> (t & 15);
        }

      lo = (lo >> 1) | (hi << 63);
      hi >>= 1;
    }
}

/* The next decimal digit of the value, or -1 after the last one */

static int big_next(FAR struct dtoa_big_s *b)
{
  for (; ; )
    {
      if (b->ingroup > 0)
        {
          static const uint16_t pow10[4] =
            {
              1, 10, 100, 1000
            };

          b->ingroup--;
          return b->group / pow10[b->ingroup] % 10;
        }

      if (!b->infrac)
        {
          if (b->limb > 0)
            {
              b->group = b->limbs[--b->limb];
              b->ingroup = 4;
              continue;
            }

          big_loadfrac(b);
        }

      if (b->nl == 0)
        {
          return -1;
        }
      else
        {
          uint32_t carry = 0;
          int i;

          for (i = b->nl - 1; i >= 0; i--)
            {
              uint32_t t = b->limbs[i] * (uint32_t)10000 + carry;

              b->limbs[i] = t & 0xffff;
              carry = t >> 16;
            }

          b->group = carry;
          b->ingroup = 4;
          while (b->nl > 0 && b->limbs[b->nl - 1] == 0)
            {
              b->nl--;
            }
        }
    }
}

/* Walk over the digits once: find the exponent, how the digits round, and
 * how many of them count.  The second walk then gives them to the output.
 * So no buffer holds the digits, and a value may have thousands of them.
 */

static int32_t big_analyze(FAR struct dtoa_big_s *b, FAR struct dtoa_s *dtoa,
                           int32_t max_digits, int32_t decimals)
{
  int32_t exp;
  int32_t want = max_digits;
  int32_t n = 0;
  int32_t nz = 0;
  int32_t non9 = -1;
  int last = 0;
  int next;
  bool rest = false;

  big_restart(b);
  exp = b->limb > 0 ? (int32_t)(b->limb - 1) * 4 + 3 : -1;

  next = big_next(b);
  while (next == 0)
    {
      exp--;
      next = big_next(b);
    }

  if (decimals >= 0)
    {
      want = (decimals < DTOA_MAX_DECIMALS ? decimals : DTOA_MAX_DECIMALS) +
             exp + 1;
      if (want < 0)
        {
          goto zero;
        }
    }

  while (n < want && next >= 0)
    {
      if (next != 9)
        {
          non9 = n;
        }

      if (next != 0)
        {
          nz = n + 1;
        }

      last = next;
      n++;
      next = big_next(b);
    }

  if (next >= 0)
    {
      int d = big_next(b);

      while (d == 0)
        {
          d = big_next(b);
        }

      rest = d > 0;
    }

  b->up = DTOA_UP_NONE;
  if (next > 5 || (next == 5 && (rest || (last & 1) != 0)))
    {
      if (non9 >= 0)
        {
          b->up = non9;
          nz = non9 + 1;
        }
      else
        {
          /* 999 becomes 1000, or no digits become 1 */

          b->up = DTOA_UP_NINES;
          nz = 1;
          exp++;
          if (n == 0 || decimals >= 0)
            {
              n++;
            }
        }
    }
  else if (n == 0)
    {
zero:
      dtoa->big = NULL;
      dtoa->digits[0] = '0';
      dtoa->digits[1] = '\0';
      dtoa->exp = 0;
      return 1;
    }

  b->n = n;
  b->nz = nz;
  dtoa->big = b;
  dtoa->exp = exp;

  big_restart(b);
  return n;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __dtoa_big_start
 *
 * Description:
 *   Set up the exact digits of a double.  limbs has DTOA_BIG_LIMBS entries,
 *   and must stay until the output ends.  dtoa->flags stays from the engine:
 *   it found the sign and that the value is finite, and not 0.
 *
 * Returned Value:
 *   The number of digits.  __dtoa_big_digit() gives them when dtoa->big is
 *   not NULL.
 *
 ****************************************************************************/

int32_t __dtoa_big_start(FAR struct dtoa_big_s *b, FAR uint16_t *limbs,
                         double x, FAR struct dtoa_s *dtoa,
                         int32_t max_digits, int32_t decimals)
{
#if DBL_MANT_DIG == 53
  uint64_t u;
#else
  uint32_t u;
#endif
  int ef;

  memcpy(&u, &x, sizeof(u));
  ef = (u >> (DBL_MANT_DIG - 1)) & (2 * DBL_MAX_EXP - 1);

  b->limbs = limbs;
  b->hi = 0;
  b->lo = u & ((1ull << (DBL_MANT_DIG - 1)) - 1);
  if (ef != 0)
    {
      b->lo |= 1ull << (DBL_MANT_DIG - 1);
    }

  b->e2 = (ef != 0 ? ef : 1) - (DBL_MAX_EXP - 1) - (DBL_MANT_DIG - 1);
  return big_analyze(b, dtoa, max_digits, decimals);
}

/****************************************************************************
 * Name: __dtoa_big_digit
 *
 * Description:
 *   Give digit i.  The index must not go down, but it may stay.
 *
 ****************************************************************************/

char __dtoa_big_digit(FAR struct dtoa_big_s *b, int32_t i)
{
  if (b->up == DTOA_UP_NINES)
    {
      return i == 0 ? '1' : '0';
    }

  if (b->up >= 0 && i > b->up)
    {
      return '0';
    }

  while (b->pos <= i)
    {
      int d = big_next(b);

      while (b->pos == 0 && d == 0)
        {
          d = big_next(b);
        }

      b->cur = d < 0 ? 0 : d;
      b->pos++;
    }

  return b->cur + '0' + (i == b->up ? 1 : 0);
}

#ifdef HAVE_DTOA_LDOUBLE
/****************************************************************************
 * Name: __dtoa_big_start_ld
 *
 * Description:
 *   The same for a long double.  limbs has DTOA_BIG_LD_LIMBS entries.  This
 *   also finds the sign, infinity, NaN and 0 itself.
 *
 ****************************************************************************/

int32_t __dtoa_big_start_ld(FAR struct dtoa_big_s *b, FAR uint16_t *limbs,
                            long double x, FAR struct dtoa_s *dtoa,
                            int32_t max_digits, int32_t decimals)
{
  bool nonfinite;
  int ef;

#  if LDBL_MANT_DIG == 113
  uint64_t w[2];

  memcpy(w, &x, sizeof(w));
  dtoa->flags = (w[DTOA_W_HI] >> 63) != 0 ? DTOA_MINUS : 0;
  ef = (w[DTOA_W_HI] >> 48) & 0x7fff;
  b->hi = w[DTOA_W_HI] & 0xffffffffffffull;
  b->lo = w[DTOA_W_LO];
  nonfinite = ef == 0x7fff;
  dtoa->flags |= nonfinite && (b->hi | b->lo) != 0 ? DTOA_NAN : 0;
  if (ef != 0 && !nonfinite)
    {
      b->hi |= 1ull << 48;
    }

  b->e2 = (int32_t)(ef != 0 ? ef : 1) - 16383 - 112;
#  else
  uint16_t se;

  memcpy(&b->lo, &x, sizeof(b->lo));
  memcpy(&se, (FAR const char *)&x + sizeof(b->lo), sizeof(se));
  dtoa->flags = (se >> 15) != 0 ? DTOA_MINUS : 0;
  ef = se & 0x7fff;
  b->hi = 0;
  nonfinite = ef == 0x7fff;
  dtoa->flags |= nonfinite && (b->lo << 1) != 0 ? DTOA_NAN : 0;
  b->e2 = (int32_t)(ef != 0 ? ef : 1) - 16383 - 63;
#  endif

  b->limbs = limbs;
  dtoa->big = NULL;
  dtoa->exp = 0;

  if (nonfinite)
    {
      if ((dtoa->flags & DTOA_NAN) == 0)
        {
          dtoa->flags |= DTOA_INF;
        }

      dtoa->digits[0] = '\0';
      return 0;
    }

  if ((b->hi | b->lo) == 0)
    {
      /* The output prints a 0 for each digit that it does not have */

      dtoa->flags |= DTOA_ZERO;
      dtoa->digits[0] = '0';
      dtoa->digits[1] = '\0';
      return 1;
    }

  return big_analyze(b, dtoa, max_digits, decimals);
}
#endif
