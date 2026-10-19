/****************************************************************************
 * libs/libm/libm/lib_exp2f.c
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

float exp2f(float x)
{
  const float ln2 = 0.693147180559945309417232121458176568F;
  float n;

  if (isnan(x))
    {
      return x;
    }

  /* Large |x|: scalbn() gives the overflow or the underflow */

  if (x > 300.0F)
    {
      x = 300.0F;
    }
  else if (x < -300.0F)
    {
      x = -300.0F;
    }

  /* 2^x = 2^n * e^(f * ln 2), with n an integer and |f| <= 0.5 */

  n = floorf(x + 0.5F);
  return scalbnf(expf((x - n) * ln2), (int)n);
}
