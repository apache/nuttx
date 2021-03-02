/****************************************************************************
 * libs/libc/fixedmath/lib_b16sin.c
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

#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define b16_P225       0x0000399a
#define b16_P405284735 0x000067c1
#define b16_1P27323954 0x000145f3

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: b16sin
 * Ref:
 * lab.polygonal.de/2007/07/18/fast-and-accurate-sinecosine-approximation/
 ****************************************************************************/

b16_t b16sin(b16_t rad)
{
  b16_t tmp1;
  b16_t tmp2;
  b16_t tmp3;

  /* Force angle into the good range */

  if (rad < -b16PI)
    {
      rad += b16TWOPI;
    }
  else if (rad > b16PI)
    {
      rad -= b16TWOPI;
    }

  /* tmp1 = 1.27323954 * rad
   * tmp2 = .405284735 * rad * rad
   */

  tmp1 = b16mulb16(b16_1P27323954, rad);
  tmp2 = b16mulb16(b16_P405284735, b16sqr(rad));

  if (rad < 0)
    {
      /* tmp3 = 1.27323954 * rad + .405284735 * rad * rad */

      tmp3 = tmp1 + tmp2;
    }
  else
    {
      /* tmp3 = 1.27323954 * rad - 0.405284735 * rad * rad */

      tmp3 = tmp1 - tmp2;
    }

  /* tmp1 = tmp3*tmp3 */

  tmp1 = b16sqr(tmp3);
  if (tmp3 < 0)
    {
      /* tmp1 = tmp3 * -tmp3 */

      tmp1 = -tmp1;
    }

  /* Return sin = .225 * (tmp3 * (+/-tmp3) - tmp3) + tmp3 */

  return b16mulb16(b16_P225, (tmp1 - tmp3)) + tmp3;
}
