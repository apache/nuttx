/****************************************************************************
 * libs/libm/libm/lib_remquof.c
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

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float remquof(float x, float y, FAR int *quo)
{
  float ax = fabsf(x);
  float ay = fabsf(y);
  float t;
  uint32_t q = 0;
  int e;

  *quo = 0;
  if (isnan(x) || isnan(y) || isinf(x) || y == 0.0F)
    {
      return (x * y) / (x * y);
    }

  if (x == 0.0F || isinf(y))
    {
      return x;
    }

  /* Long division: subtract ay * 2^e from ax for e down to 0.  Each step
   * is exact, so the remainder is exact.  Keep the low quotient bits.
   */

  e = ilogbf(ax) - ilogbf(ay);
  if (e >= 0)
    {
      t = scalbnf(ay, e);
      for (; ; )
        {
          if (ax >= t)
            {
              ax -= t;
              if (e < 31)
                {
                  q |= (uint32_t)1 << e;
                }
            }

          if (e-- == 0)
            {
              break;
            }

          t *= 0.5F;
        }
    }

  /* Round the quotient to the nearest integer, ties to even */

  if (ax + ax > ay || (ax + ax == ay && (q & 1) != 0))
    {
      ax -= ay;
      q++;
    }

  q &= 0x7fffffff;
  *quo = (x < 0.0F) != (y < 0.0F) ? -(int)q : (int)q;
  return x < 0.0F ? -ax : ax;
}
