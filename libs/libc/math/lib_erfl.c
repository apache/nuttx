/****************************************************************************
 * libs/libc/math/lib_erfl.c
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

#define  A1     0.254829592
#define  A2   (-0.284496736)
#define  A3     1.421413741
#define  A4   (-1.453152027)
#define  A5     1.061405429
#define  P      0.3275911

#ifdef CONFIG_HAVE_LONG_DOUBLE
long double erfl(long double x)
{
  /* This implementation comes from the Handbook of Mathmatical Functions
   * The implementations in this book are not protected by copyright.
   * erf comes from formula 7.1.26
   */

  long double t;
  long double z;

  z = fabsl(x);
  t = 1.0 / (1.0 + P * z);
  t = 1.0 -
      (((((A5 * t + A4) * t) + A3) * t + A2) * t + A1) * t * expl(-z * z);
  return copysignl(t, x);
}
#endif
