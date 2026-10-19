/****************************************************************************
 * libs/libm/libm/lib_hypotf.c
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

float hypotf(float x, float y)
{
  float a = fabsf(x);
  float b = fabsf(y);
  float t;

  if (isinf(a) || isinf(b))
    {
      return INFINITY_F;
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

  if (b == 0.0F)
    {
      return a;
    }

  /* a * sqrt(1 + (b / a)^2) does not overflow before the result does */

  t = b / a;
  return a * sqrtf(1.0F + t * t);
}
