/****************************************************************************
 * libs/libnx/nxglib/nxglib_circletraps.c
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

#include <string.h>
#include <errno.h>
#include <fixedmath.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Trigonometry */

#define SIN_0p0        0         /* sin(0) = 0 */
#define COS_0p0        1         /* cos(0) = 1 */
#define SIN_22p5       25080     /* sin(22.5) = 25080 / 65536 */
#define COS_22p5       60547     /* cos(22.5) = 60547 / 65536 */
#define SIN_45p0       46341     /* sin(45) = 46341 / 65536 */
#define COS_45p0       SIN_45p0  /* cos(45) = sin(45) */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_circletraps
 *
 * Description:
 *   Given a description of a a circle, return 8 trapezoids that can be
 *   used to fill the circle by nx_fillcircle() and other interfaces.
 *
 * Input Parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 8 trapezoids where
 *            the circle description will be returned.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxgl_circletraps(FAR const struct nxgl_point_s *center,
                      nxgl_coord_t radius,
                      FAR struct nxgl_trapezoid_s *circle)
{
  nxgl_coord_t xoffs;
  nxgl_coord_t yoffs;

  circle[0].top.x1      = itob16(center->x);
  circle[0].top.x2      = circle[0].top.x1;
  circle[0].top.y       = center->y - radius;

  circle[7].bot.x1      = circle[0].top.x1;
  circle[7].bot.x2      = circle[0].top.x1;
  circle[7].bot.y       = center->y + radius;

  circle[3].bot.x1      = itob16(center->x - radius);
  circle[3].bot.x2      = itob16(center->x + radius);
  circle[3].bot.y       = center->y;

  circle[4].top.x1      = circle[3].bot.x1;
  circle[4].top.x2      = circle[3].bot.x2;
  circle[4].top.y       = circle[3].bot.y;

  /* Now 22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, and 337.5 */

  xoffs = b16toi(b16muli(COS_22p5, radius) + b16HALF);
  yoffs = b16toi(b16muli(SIN_22p5, radius) + b16HALF);

  circle[2].bot.x1      = itob16(center->x - xoffs);
  circle[2].bot.x2      = itob16(center->x + xoffs);
  circle[2].bot.y       = center->y - yoffs;

  circle[3].top.x1      = circle[2].bot.x1;
  circle[3].top.x2      = circle[2].bot.x2;
  circle[3].top.y       = circle[2].bot.y;

  circle[4].bot.x1      = circle[2].bot.x1;
  circle[4].bot.x2      = circle[2].bot.x2;
  circle[4].bot.y       = center->y + yoffs;

  circle[5].top.x1      = circle[4].bot.x1;
  circle[5].top.x2      = circle[4].bot.x2;
  circle[5].top.y       = circle[4].bot.y;

  circle[0].bot.x1      = itob16(center->x - yoffs);
  circle[0].bot.x2      = itob16(center->x + yoffs);
  circle[0].bot.y       = center->y - xoffs;

  circle[1].top.x1      = circle[0].bot.x1;
  circle[1].top.x2      = circle[0].bot.x2;
  circle[1].top.y       = circle[0].bot.y;

  circle[6].bot.x1      = circle[1].top.x1;
  circle[6].bot.x2      = circle[1].top.x2;
  circle[6].bot.y       = center->y + xoffs;

  circle[7].top.x1      = circle[6].bot.x1;
  circle[7].top.x2      = circle[6].bot.x2;
  circle[7].top.y       = circle[6].bot.y;

  /* Finally, 45.0, 135.0, 225.0, 315.0 */

  xoffs = b16toi(b16muli(COS_45p0, radius) + b16HALF);

  circle[1].bot.x1      = itob16(center->x - xoffs);
  circle[1].bot.x2      = itob16(center->x + xoffs);
  circle[1].bot.y       = center->y - xoffs;

  circle[2].top.x1      = circle[1].bot.x1;
  circle[2].top.x2      = circle[1].bot.x2;
  circle[2].top.y       = circle[1].bot.y;

  circle[5].bot.x1      = circle[1].bot.x1;
  circle[5].bot.x2      = circle[1].bot.x2;
  circle[5].bot.y       = center->y + xoffs;

  circle[6].top.x1      = circle[5].bot.x1;
  circle[6].top.x2      = circle[5].bot.x2;
  circle[6].top.y       = circle[5].bot.y;
}
