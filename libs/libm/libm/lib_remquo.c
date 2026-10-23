/****************************************************************************
 * libs/libm/libm/lib_remquo.c
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double remquo(double x, double y, FAR int *quo)
{
  double ax = fabs(x);
  double ay = fabs(y);
  double t;
  uint32_t q = 0;
  int e;

  *quo = 0;
  if (isnan(x) || isnan(y) || isinf(x) || y == 0.0)
    {
      return (x * y) / (x * y);
    }

  if (x == 0.0 || isinf(y))
    {
      return x;
    }

  /* Long division: subtract ay * 2^e from ax for e down to 0.  Each step
   * is exact, so the remainder is exact.  Keep the low quotient bits.
   */

  e = ilogb(ax) - ilogb(ay);
  if (e >= 0)
    {
      t = scalbn(ay, e);
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

          t *= 0.5;
        }
    }

  /* Round the quotient to the nearest integer, ties to even */

  if (ax + ax > ay || (ax + ax == ay && (q & 1) != 0))
    {
      ax -= ay;
      q++;
    }

  q &= 0x7fffffff;
  *quo = (x < 0.0) != (y < 0.0) ? -(int)q : (int)q;
  return x < 0.0 ? -ax : ax;
}
#endif
