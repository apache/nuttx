/****************************************************************************
 * libs/libm/libm/lib_log1pf.c
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

#include <float.h>
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float log1pf(float x)
{
  float s;
  float s2;
  float t;
  float term;
  float sum;
  int k;

  if (isnan(x) || x == INFINITY_F)
    {
      return x;
    }

  if (x == -1.0F)
    {
      return -HUGE_VALF;
    }

  if (x < -1.0F)
    {
      return (x - x) / (x - x);
    }

  if (x <= -0.5F || x >= 1.0F)
    {
      return logf(1.0F + x);
    }

  if (1.0F + x == 1.0F)
    {
      return x;
    }

  /* log(1 + x) = 2 * atanh(s) = 2 * (s + s^3 / 3 + s^5 / 5 + ...), with
   * s = x / (2 + x) and |s| < 1 / 3
   */

  s = x / (2.0F + x);
  s2 = s * s;
  t = s;
  sum = s;
  for (k = 3; ; k += 2)
    {
      t *= s2;
      term = t / k;
      if (fabsf(term) <= fabsf(sum) * FLT_EPSILON * 0.25F)
        {
          break;
        }

      sum += term;
    }

  return 2.0F * sum;
}
