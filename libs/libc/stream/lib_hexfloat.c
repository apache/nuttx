/****************************************************************************
 * libs/libc/stream/lib_hexfloat.c
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

#include <string.h>

#include "lib_hexfloat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A double has 64 bits, or 32 bits on some small CPUs */

#if DBL_MANT_DIG > 24
typedef uint64_t hexfloat_dbits_t;
#else
typedef uint32_t hexfloat_dbits_t;
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define HEXFLOAT_HI 0
#  define HEXFLOAT_LO 1
#else
#  define HEXFLOAT_HI 1
#  define HEXFLOAT_LO 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* hi and lo hold the bits after the point, from the top.  All 64-bit shifts
 * have a constant count, so a 32-bit CPU needs no libgcc call.
 */

static void hexfloat_convert(FAR struct hexfloat_s *hex, bool neg, int exp,
                             int expmax, uint64_t hi, uint64_t lo, int ndigs,
                             int prec, bool upper)
{
  int i;

  hex->flags = neg ? HEXFLOAT_MINUS : 0;
  hex->digits[0] = 1;

  if (exp == expmax)
    {
      hex->flags |= (hi | lo) != 0 ? HEXFLOAT_NAN : HEXFLOAT_INF;
      return;
    }

  if (exp == 0)
    {
      /* Zero, or a subnormal value with the exponent of the smallest normal
       * value.
       */

      hex->digits[0] = 0;
      exp = (hi | lo) != 0;
    }

  hex->exp = (hex->digits[0] | exp) != 0 ? exp - expmax / 2 : 0;

  for (i = 1; i <= ndigs; i++)
    {
      hex->digits[i] = hi >> 60;
      hi = (hi << 4) | (lo >> 60);
      lo <<= 4;
    }

  if (prec >= 0 && prec < ndigs)
    {
      unsigned char rest = 0;

      /* Round to nearest, ties to even */

      for (i = prec + 2; i <= ndigs; i++)
        {
          rest |= hex->digits[i];
        }

      if (hex->digits[prec + 1] > 8 ||
          (hex->digits[prec + 1] == 8 &&
           (rest != 0 || (hex->digits[prec] & 1) != 0)))
        {
          for (i = prec; i > 0 && hex->digits[i] == 15; i--)
            {
              hex->digits[i] = 0;
            }

          hex->digits[i]++;
        }

      ndigs = prec;
    }
  else if (prec < 0)
    {
      while (ndigs > 0 && hex->digits[ndigs] == 0)
        {
          ndigs--;
        }
    }

  hex->ndigs = ndigs;
  for (i = 0; i <= ndigs; i++)
    {
      hex->digits[i] += hex->digits[i] < 10 ? '0' :
                        (upper ? 'A' : 'a') - 10;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __hexfloat_engine(double x, FAR struct hexfloat_s *hex, int prec,
                       bool upper)
{
  hexfloat_dbits_t u;

  memcpy(&u, &x, sizeof(u));
  hexfloat_convert(hex, u >> (sizeof(u) * 8 - 1),
                   (u >> (DBL_MANT_DIG - 1)) & (2 * DBL_MAX_EXP - 1),
                   2 * DBL_MAX_EXP - 1,
                   (uint64_t)u << (64 - DBL_MANT_DIG + 1), 0,
                   (DBL_MANT_DIG + 2) / 4, prec, upper);
}

#ifdef HAVE_HEXFLOAT_LDOUBLE
void __hexfloat_engine_ld(long double x, FAR struct hexfloat_s *hex,
                          int prec, bool upper)
{
#  if LDBL_MANT_DIG == 113
  uint64_t w[2];

  memcpy(w, &x, sizeof(w));
  hexfloat_convert(hex, w[HEXFLOAT_HI] >> 63,
                   (w[HEXFLOAT_HI] >> 48) & 0x7fff, 0x7fff,
                   (w[HEXFLOAT_HI] << 16) | (w[HEXFLOAT_LO] >> 48),
                   w[HEXFLOAT_LO] << 16, 28, prec, upper);
#  else
  uint64_t mant;
  uint16_t se;

  /* The 80-bit format stores the bit before the point */

  memcpy(&mant, &x, sizeof(mant));
  memcpy(&se, (FAR const char *)&x + sizeof(mant), sizeof(se));
  hexfloat_convert(hex, se >> 15, se & 0x7fff, 0x7fff, mant << 1, 0, 16,
                   prec, upper);
#  endif
}
#endif
