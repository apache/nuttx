/****************************************************************************
 * libs/libm/libm/lib_nextafter.c
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
double nextafter(double x, double y)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  if (isnan(x) || isnan(y))
    {
      return x + y;
    }

  if (x == y)
    {
      return y;
    }

  if (x == 0.0)
    {
      /* The smallest subnormal number, with the sign of y */

      u.i = 1;
      return y > 0.0 ? u.f : -u.f;
    }

  /* The next number has the next bit pattern in the direction of y */

  u.f = x;
  if ((x < y) == (x > 0.0))
    {
      u.i++;
    }
  else
    {
      u.i--;
    }

  return u.f;
}
#endif
