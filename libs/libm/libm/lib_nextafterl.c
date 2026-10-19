/****************************************************************************
 * libs/libm/libm/lib_nextafterl.c
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

#ifdef CONFIG_HAVE_LONG_DOUBLE
#if LDBL_MANT_DIG == DBL_MANT_DIG
long double nextafterl(long double x, long double y)
{
  return nextafter(x, y);
}
#else
long double nextafterl(long double x, long double y)
{
  long double ulp;
  int e;

  if (isnan(x) || isnan(y))
    {
      return x + y;
    }

  if (x == y)
    {
      return y;
    }

  if (x == 0.0L)
    {
      ulp = scalbnl(1.0L, LDBL_MIN_EXP - LDBL_MANT_DIG);
      return y > 0.0L ? ulp : -ulp;
    }

  if (isinf(x))
    {
      return x > 0.0L ? LDBL_MAX : -LDBL_MAX;
    }

  /* x = m * 2^e with |m| in [0.5, 1): the distance to the next number is
   * 2^(e - LDBL_MANT_DIG), half of it below a power of two, and not less
   * than the distance between subnormal numbers.
   */

  if (fabsl(frexpl(x, &e)) == 0.5L && (y > x) != (x > 0.0L) &&
      e > LDBL_MIN_EXP)
    {
      e--;
    }

  if (e < LDBL_MIN_EXP)
    {
      e = LDBL_MIN_EXP;
    }

  ulp = scalbnl(1.0L, e - LDBL_MANT_DIG);
  return y > x ? x + ulp : x - ulp;
}
#endif
#endif
