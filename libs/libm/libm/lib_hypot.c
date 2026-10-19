/****************************************************************************
 * libs/libm/libm/lib_hypot.c
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
double hypot(double x, double y)
{
  double a = fabs(x);
  double b = fabs(y);
  double t;

  if (isinf(a) || isinf(b))
    {
      return INFINITY;
    }

  if (isnan(a) || isnan(b))
    {
      return a + b;
    }

  if (a < b)
    {
      t = a;
      a = b;
      b = t;
    }

  if (b == 0.0)
    {
      return a;
    }

  /* a * sqrt(1 + (b / a)^2) does not overflow before the result does */

  t = b / a;
  return a * sqrt(1.0 + t * t);
}
#endif
