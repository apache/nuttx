/****************************************************************************
 * libs/libc/fixedmath/lib_b16atan2.c
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

#define B16_C1     0x00000373 /* 0.013480470 */
#define B16_C2     0x00000eb7 /* 0.057477314 */
#define B16_C3     0x00001f0a /* 0.121239071 */
#define B16_C4     0x00003215 /* 0.195635925 */
#define B16_C5     0x0000553f /* 0.332994597 */
#define B16_C6     0x00010000 /* 0.999995630 */
#define B16_HALFPI 0x00019220 /* 1.570796327 */
#define B16_PI     0x00032440 /* 3.141592654 */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#  define ABS(a)   ((a) < 0 ? -(a) : (a))
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: b16atan2
 *
 * Description:
 *   atan2 calculates the arctangent of y/x.  (Based on a algorithm I saw
 *   posted on the internet... now I have lost the link -- sorry).
 *
 ****************************************************************************/

b16_t b16atan2(b16_t y, b16_t x)
{
  b16_t t0;
  b16_t t1;
  b16_t t2;
  b16_t t3;

  t2 = ABS(x);
  t1 = ABS(y);
  t0 = MAX(t2, t1);
  t1 = MIN(t2, t1);
  t2 = ub16inv(t0);
  t2 = b16mulb16(t1, t2);

  t3 = b16mulb16(t2, t2);
  t0 =                   - B16_C1;
  t0 = b16mulb16(t0, t3) + B16_C2;
  t0 = b16mulb16(t0, t3) - B16_C3;
  t0 = b16mulb16(t0, t3) + B16_C4;
  t0 = b16mulb16(t0, t3) - B16_C5;
  t0 = b16mulb16(t0, t3) + B16_C6;
  t2 = b16mulb16(t0, t2);

  t2 = (ABS(y) > ABS(x)) ? B16_HALFPI - t2 : t2;
  t2 = (x < 0) ?  B16_PI - t2 : t2;
  t2 = (y < 0) ? -t2 : t2;

  return t2;
}
