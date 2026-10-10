/****************************************************************************
 * tools/test_fmaf.c
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
 * Compile lib_fmaf.c separately with -Dfmaf=nuttx_fmaf -frounding-math
 * -ffp-contract=off, then link this file and that object with -lm:
 *
 *   cc -O2 -frounding-math -ffp-contract=off -Dfmaf=nuttx_fmaf \
 *      -c libs/libm/libm/lib_fmaf.c -o lib_fmaf.o
 *   cc -O2 -frounding-math -ffp-contract=off tools/test_fmaf.c lib_fmaf.o \
 *      -o test_fmaf -lm && ./test_fmaf
 *
 * Compare with host libm in all four rounding modes. NaN payloads may vary;
 * finite values, signed zeros and required exception flags must match.
 * The exit status is zero if all cases match.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

float nuttx_fmaf(float x, float y, float z);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t next_bits(uint32_t *state)
{
  *state = *state * UINT32_C(1664525) + UINT32_C(1013904223);
  return *state;
}

static float from_bits(uint32_t bits)
{
  float value;

  memcpy(&value, &bits, sizeof(value));
  return value;
}

static uint32_t to_bits(float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

static int check(uint32_t x, uint32_t y, uint32_t z, int rounding)
{
  float a = from_bits(x);
  float b = from_bits(y);
  float c = from_bits(z);
  int mask = FE_INVALID | FE_OVERFLOW | FE_UNDERFLOW | FE_INEXACT;
  int expected_flags;
  int actual_flags;
  float expected;
  float actual;

  feclearexcept(FE_ALL_EXCEPT);
  expected = fmaf(a, b, c);
  expected_flags = fetestexcept(mask);
  feclearexcept(FE_ALL_EXCEPT);
  actual = nuttx_fmaf(a, b, c);
  actual_flags = fetestexcept(mask);

  /* C permits either invalid-flag choice for infinity times zero plus a
   * quiet NaN. A scalar multiply/add raises it while a hardware FMA can
   * give the NaN precedence; both return the required NaN result.
   */

  if ((z & UINT32_C(0x7fc00000)) == UINT32_C(0x7fc00000) &&
      (((x & UINT32_C(0x7fffffff)) == 0 &&
        (y & UINT32_C(0x7fffffff)) == UINT32_C(0x7f800000)) ||
       ((y & UINT32_C(0x7fffffff)) == 0 &&
        (x & UINT32_C(0x7fffffff)) == UINT32_C(0x7f800000))))
    {
      actual_flags &= ~FE_INVALID;
      expected_flags &= ~FE_INVALID;
    }

  if (!(isnan(actual) && isnan(expected)) &&
      to_bits(actual) != to_bits(expected))
    {
      fprintf(stderr, "fmaf result: round=%d x=%08x y=%08x z=%08x "
              "got=%08x expected=%08x\n", rounding, x, y, z,
              to_bits(actual), to_bits(expected));
      return 1;
    }

  if (actual_flags != expected_flags)
    {
      fprintf(stderr, "fmaf exceptions: round=%d x=%08x y=%08x z=%08x "
              "got=%x expected=%x\n", rounding, x, y, z,
              actual_flags, expected_flags);
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  static const int rounding[] =
  {
    FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO
  };

  static const uint32_t boundary[] =
  {
    0, 0x80000000, 1, 0x80000001, 0x007fffff, 0x807fffff,
    0x00800000, 0x80800000, 0x3f000000, 0xbf000000,
    0x3f800000, 0xbf800000, 0x3f800001, 0xbf800001,
    0x7f7fffff, 0xff7fffff, 0x7f800000, 0xff800000,
    0x7fc00001, 0x7f800001
  };

  /* Products on either side of a binary32 halfway value must not acquire
   * a false tie while rounding via binary64, including subnormal outputs.
   */

  static const uint32_t ties[][3] =
  {
    { 0x3f800001, 0x3f800001, 0xbf800002 },
    { 0x007fffff, 0x3f800001, 0x00800000 },
    { 0x00000001, 0x3f000000, 0x00000001 },
    { 0x00000001, 0x00000001, 0x00800000 },
    { 0x80000001, 0x00000001, 0x00800000 },
    { 0x3f800000, 0x3f800000, 0x33800000 },
    { 0x3f800000, 0x3f800000, 0x33800001 },
    { 0x3f800000, 0x3f800000, 0x337fffff },
  };

  /* Mantissas (in units of the result's last place) for the double
   * rounding traps below: even and odd, all ones, and subnormal.
   */

  static const uint32_t trapmant[] =
  {
    1, 3, 0x7fffff, 0x800000, 0x800001, 0x800002, 0x555555, 0xaaaaaa,
    0xfffffe, 0xffffff
  };

  const unsigned int nrounding = sizeof(rounding) / sizeof(rounding[0]);
  const unsigned int nties = sizeof(ties) / sizeof(ties[0]);
  const unsigned int nboundary = sizeof(boundary) / sizeof(boundary[0]);
  const unsigned int ntrapmant = sizeof(trapmant) / sizeof(trapmant[0]);
  int original = fegetround();
  uint64_t count = 0;
  int ret = 1;

  for (unsigned int r = 0; r < nrounding; r++)
    {
      uint32_t state = 0x230;

      if (fesetround(rounding[r]))
        {
          goto done;
        }

      for (unsigned int i = 0; i < nties; i++)
        {
          if (check(ties[i][0], ties[i][1], ties[i][2], rounding[r]))
            {
              goto done;
            }

          count++;
        }

      for (unsigned int i = 0; i < nboundary; i++)
        {
          for (unsigned int j = 0; j < nboundary; j++)
            {
              for (unsigned int k = 0; k < nboundary; k++)
                {
                  if (check(boundary[i], boundary[j], boundary[k],
                            rounding[r]))
                    {
                      goto done;
                    }

                  count++;
                }
            }
        }

      /* Double rounding traps. With x = (1 + 2^-23) 2^a and
       * y = (1 - 2^-23) 2^b, the product is 2^(e-1) (1 - 2^-46) for
       * e = a + b + 1: exact in binary64, and just below half a unit in
       * the last place of z = m 2^e. The exact sum is therefore just
       * below (or, if z has the opposite sign, just above) a binary32
       * halfway value, but its binary64 rounding is that halfway value.
       * Converting that to binary32 picks the even neighbour, which is
       * the wrong one half of the time. This is what the halfway
       * correction in fmaf() is for; random inputs reach it with a
       * probability near 2^-29 and the cases above only by accident.
       */

      for (int e = -149; e <= 104; e++)
        {
          const int a = (e - 1) / 2;
          const int b = e - 1 - a;
          const float x = ldexpf(1.0f + FLT_EPSILON, a);
          const float y = ldexpf(1.0f - FLT_EPSILON, b);

          for (unsigned int m = 0; m < ntrapmant; m++)
            {
              const float z = ldexpf((float)trapmant[m], e);

              for (unsigned int s = 0; s < 4; s++)
                {
                  uint32_t xbits = to_bits(x) ^ ((s & 1) ? 0x80000000 : 0);
                  uint32_t zbits = to_bits(z) ^ ((s & 2) ? 0x80000000 : 0);

                  if (check(xbits, to_bits(y), zbits, rounding[r]))
                    {
                      goto done;
                    }

                  count++;
                }
            }
        }

      for (unsigned int i = 0; i < 1000000; i++)
        {
          uint32_t x = next_bits(&state);
          uint32_t y = next_bits(&state);
          uint32_t z = next_bits(&state);

          if (check(x, y, z, rounding[r]))
            {
              goto done;
            }

          count++;
        }
    }

  printf("fmaf: %llu cases match host libm bits and exception flags "
         "in four rounding modes\n", (unsigned long long)count);
  ret = 0;
done:
  fesetround(original);
  return ret;
}
