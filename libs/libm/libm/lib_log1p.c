/****************************************************************************
 * libs/libm/libm/lib_log1p.c
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

#include <float.h>
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double log1p(double x)
{
  double s;
  double s2;
  double t;
  double term;
  double sum;
  int k;

  if (isnan(x) || x == INFINITY)
    {
      return x;
    }

  if (x == -1.0)
    {
      return -HUGE_VAL;
    }

  if (x < -1.0)
    {
      return (x - x) / (x - x);
    }

  if (x <= -0.5 || x >= 1.0)
    {
      return log(1.0 + x);
    }

  if (1.0 + x == 1.0)
    {
      return x;
    }

  /* log(1 + x) = 2 * atanh(s) = 2 * (s + s^3 / 3 + s^5 / 5 + ...), with
   * s = x / (2 + x) and |s| < 1 / 3
   */

  s = x / (2.0 + x);
  s2 = s * s;
  t = s;
  sum = s;
  for (k = 3; ; k += 2)
    {
      t *= s2;
      term = t / k;
      if (fabs(term) <= fabs(sum) * DBL_EPSILON * 0.25)
        {
          break;
        }

      sum += term;
    }

  return 2.0 * sum;
}
#endif
