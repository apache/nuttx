/****************************************************************************
 * libs/libnx/nxglib/nxglib_nonintersecting.c
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

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_nonintersecting
 *
 * Description:
 *   Return the regions of rectangle rect 1 that do not intersect with
 *   rect2.  This may be up to founr rectangles some of which may be
 *   degenerate (and can be picked off with nxgl_nullrect)
 *
 ****************************************************************************/

void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4],
                          FAR const struct nxgl_rect_s *rect1,
                          FAR const struct nxgl_rect_s *rect2)
{
  struct nxgl_rect_s intersection;

  /* Get the intersection of the two rectangles */

  nxgl_rectintersect(&intersection, rect1, rect2);

  /* Then return the four rectangles representing the regions NON included
   * in the intersection.  Some of these rectangles may be invalid (zero
   * area), but those can be picked off using nxgl_nullrect()
   *
   *  rect1.pt1
   *   +-------------------------+
   *   |         rect2.pt1       |
   *   |         int.pt1         |
   *   |         +-------------------------+
   *   |         |               |         |
   *   |         |               |         |
   *   +-------------------------+         |
   *             |               rect1.pt2 |
   *             |               int.pt2   |
   *             +-------------------------+
   *                                       rect2.pt2
   *             rect1.pt1
   *             +-------------------------+
   *   rect2.pt1 |int.pt1                  |
   *   +---------+---------------+         |
   *   |         |               |         |
   *   |         |               |         |
   *   |         |               |int.pt2  |
   *   |         +---------------+---------+
   *   |                         |         rect1.pt2
   *   +-------------------------+
   *                             rect2.pt2
   *   rect2.pt1
   *   +-------------------------+
   *   |         rect1.pt1       |
   *   |         int.pt1         |
   *   |         +-------------------------+
   *   |         |               |         |
   *   |         |               |         |
   *   |         |               |         |
   *   +---------+---------------+         |
   *             |               rect2.pt2 |
   *             |               int.pt2   |
   *             +-------------------------+
   *                                       rect1.pt2
   */

  result[NX_TOP_NDX].pt1.x    = rect1->pt1.x;
  result[NX_TOP_NDX].pt1.y    = rect1->pt1.y;
  result[NX_TOP_NDX].pt2.x    = rect1->pt2.x;
  result[NX_TOP_NDX].pt2.y    = intersection.pt1.y - 1;

  result[NX_BOTTOM_NDX].pt1.x = rect1->pt1.x;
  result[NX_BOTTOM_NDX].pt1.y = intersection.pt2.y + 1;
  result[NX_BOTTOM_NDX].pt2.x = rect1->pt2.x;
  result[NX_BOTTOM_NDX].pt2.y = rect1->pt2.y;

  result[NX_LEFT_NDX].pt1.x   = rect1->pt1.x;
  result[NX_LEFT_NDX].pt1.y   = intersection.pt1.y;
  result[NX_LEFT_NDX].pt2.x   = intersection.pt1.x - 1;
  result[NX_LEFT_NDX].pt2.y   = intersection.pt2.y;

  result[NX_RIGHT_NDX].pt1.x  = intersection.pt2.x + 1;
  result[NX_RIGHT_NDX].pt1.y  = intersection.pt1.y;
  result[NX_RIGHT_NDX].pt2.x  = rect1->pt2.x;
  result[NX_RIGHT_NDX].pt2.y  = intersection.pt2.y;
}
