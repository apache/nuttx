/****************************************************************************
 * libs/libm/libm/lib_exp2.c
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
double exp2(double x)
{
  const double ln2 = 0.693147180559945309417232121458176568;
  double n;

  if (isnan(x))
    {
      return x;
    }

  /* Large |x|: scalbn() gives the overflow or the underflow */

  if (x > 2200.0)
    {
      x = 2200.0;
    }
  else if (x < -2200.0)
    {
      x = -2200.0;
    }

  /* 2^x = 2^n * e^(f * ln 2), with n an integer and |f| <= 0.5 */

  n = floor(x + 0.5);
  return scalbn(exp((x - n) * ln2), (int)n);
}
#endif
