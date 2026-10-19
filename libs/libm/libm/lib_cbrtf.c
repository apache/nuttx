/****************************************************************************
 * libs/libm/libm/lib_cbrtf.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float cbrtf(float x)
{
  float m;
  float y;
  int e;
  int r;
  int i;

  if (x == 0.0F || isnan(x) || isinf(x))
    {
      return x;
    }

  /* |x| = m * 2^e: move e % 3 into m, then cbrt(|x|) = cbrt(m) * 2^(e / 3)
   * with m in [0.5, 4).
   */

  m = frexpf(fabsf(x), &e);
  r = e % 3;
  if (r < 0)
    {
      r += 3;
    }

  m = scalbnf(m, r);
  e = (e - r) / 3;

  /* Newton iterations from a linear first guess (error < 12%) */

  y = 0.68F + 0.227F * m;
  for (i = 0; i < 4; i++)
    {
      y -= (y - m / (y * y)) / 3.0F;
    }

  return copysignf(scalbnf(y, e), x);
}
