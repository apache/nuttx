/****************************************************************************
 * libs/libm/libm/lib_cbrt.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double cbrt(double x)
{
  double m;
  double y;
  int e;
  int r;
  int i;

  if (x == 0.0 || isnan(x) || isinf(x))
    {
      return x;
    }

  /* |x| = m * 2^e: move e % 3 into m, then cbrt(|x|) = cbrt(m) * 2^(e / 3)
   * with m in [0.5, 4).
   */

  m = frexp(fabs(x), &e);
  r = e % 3;
  if (r < 0)
    {
      r += 3;
    }

  m = scalbn(m, r);
  e = (e - r) / 3;

  /* Newton iterations from a linear first guess (error < 12%) */

  y = 0.68 + 0.227 * m;
  for (i = 0; i < 6; i++)
    {
      y -= (y - m / (y * y)) / 3.0;
    }

  return copysign(scalbn(y, e), x);
}
#endif
