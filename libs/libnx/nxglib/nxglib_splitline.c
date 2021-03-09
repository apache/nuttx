/****************************************************************************
 * libs/libnx/nxglib/nxglib_splitline.c
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
#include <inttypes.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct b16point_s
{
  b16_t x;
  b16_t y;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static b16_t nxgl_interpolate(b16_t x, b16_t dy, b16_t dxdy)
{
  b16_t dx = b16mulb16(dy, dxdy);
  return x + dx;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_splitline
 *
 * Description:
 *   In the general case, a line with width can be represented as a
 *   parallelogram with a triangle at the top and bottom.  Triangles and
 *   parallelograms are both degenerate versions of a trapeziod.  This
 *   function breaks a wide line into triangles and trapezoids.  This
 *   function also detects other degenerate cases:
 *
 *   1. If y1 == y2 then the line is horizontal and is better represented
 *      as a rectangle.
 *   2. If x1 == x2 then the line is vertical and also better represented
 *      as a rectangle.
 *   3. If the width of the line is 1, then there are no triangles at the
 *      top and bottome (this may also be the case if the width is narrow
 *      and the line is near vertical).
 *   4. If the line is oriented is certain angles, it may consist only of
 *      the upper and lower triangles with no trapezoid in between.  In
 *      this case, 3 trapezoids will be returned, but traps[1] will be
 *      degenerate.
 *
 * Input Parameters:
 *   vector - A pointer to the vector described the line to be drawn.
 *   traps  - A pointer to a array of trapezoids (size 3).
 *   rect   - A pointer to a rectangle.
 *
 * Returned Value:
 *   0: Line successfully broken up into three trapezoids.  Values in
 *      traps[0], traps[1], and traps[2] are valid.
 *   1: Line successfully represented by one trapezoid. Value in traps[1]
 *      is valid.
 *   2: Line successfully represented by one rectangle. Value in rect is
 *      valid
 *  <0: On errors, a negated errno value is returned.
 *
 ****************************************************************************/

int nxgl_splitline(FAR struct nxgl_vector_s *vector,
                   FAR struct nxgl_trapezoid_s *traps,
                   FAR struct nxgl_rect_s *rect,
                   nxgl_coord_t linewidth)
{
  struct nxgl_vector_s line;
  nxgl_coord_t iheight;
  nxgl_coord_t iwidth;
  nxgl_coord_t iyoffset;
  struct b16point_s quad[4];
  b16_t b16xoffset;
  b16_t b16yoffset;
  b16_t b16dxdy;
  b16_t angle;
  b16_t cosangle;
  b16_t sinangle;
  b16_t b16x;
  b16_t b16y;

  ginfo("vector: (%d,%d)->(%d,%d) linewidth: %d\n",
        vector->pt1.x, vector->pt1.y, vector->pt2.x, vector->pt2.y,
        linewidth);

  /* First, check the linewidth */

  if (linewidth < 1)
    {
      return -EINVAL;
    }

  /* Then make sure that the start position of the line is above the end
   * position of the line... in raster order.
   */

  if (vector->pt1.y < vector->pt2.y)
    {
      /* Vector is already in raster order */

      memcpy(&line, vector, sizeof(struct nxgl_vector_s));
    }
  else if (vector->pt1.y > vector->pt2.y)
    {
      /* Swap the top and bottom */

      line.pt1.x = vector->pt2.x;
      line.pt1.y = vector->pt2.y;
      line.pt2.x = vector->pt1.x;
      line.pt2.y = vector->pt1.y;
    }
  else /* if (vector->pt1.y == vector->pt2.y) */
    {
      /* First degenerate case:  The line is horizontal. */

      if (vector->pt1.x < vector->pt2.x)
        {
          rect->pt1.x = vector->pt1.x;
          rect->pt2.x = vector->pt2.x;
        }
      else
        {
          rect->pt1.x = vector->pt2.x;
          rect->pt2.x = vector->pt1.x;
        }

      /* The height of the rectangle is the width of the line, half above
       * and half below.
       */

      rect->pt1.y = vector->pt1.y - (linewidth >> 1);
      rect->pt2.y = rect->pt1.y + linewidth - 1;

      ginfo("Horizontal rect: (%d,%d),(%d,%d)\n",
            rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y);

      return 2;
    }

  /* Check if the line is vertical */

  if (line.pt1.x == line.pt2.x)
    {
      /* Second degenerate case:  The line is vertical. */

      rect->pt1.y = line.pt1.y;
      rect->pt2.y = line.pt2.y;

      rect->pt1.x = line.pt1.x - (linewidth >> 1);
      rect->pt2.x = rect->pt1.x + linewidth - 1;

      ginfo("Vertical rect: (%d,%d),(%d,%d)\n",
            rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y);

      return 2;
    }

  /* The final degenerate case */

  if (linewidth == 1 &&
      abs(line.pt2.x - line.pt1.x) < (line.pt2.y - line.pt1.y))
    {
      /* A close to vertical line of width 1 is basically
       * a single parallelogram of width 1.
       */

      traps[1].top.x1 = itob16(line.pt1.x);
      traps[1].top.x2 = traps[1].top.x1;
      traps[1].top.y  = line.pt1.y;

      traps[1].bot.x1 = itob16(line.pt2.x);
      traps[1].bot.x2 = traps[1].bot.x1;
      traps[1].bot.y  = line.pt2.y;

      ginfo("Vertical traps[1]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
            "(%08" PRIx32 ",%08" PRIx32 ", %d)\n",
            traps[1].top.x1, traps[1].top.x2, traps[1].top.y,
            traps[1].bot.x1, traps[1].bot.x2, traps[1].bot.y);

      return 1;
    }
  else if (linewidth == 1)
    {
      b16_t pixels_per_row;

      /* Close to horizontal line of width 1 */

      pixels_per_row  = itob16(line.pt2.x - line.pt1.x) /
                        (line.pt2.y - line.pt1.y);

      traps[1].top.x1 = itob16(line.pt1.x);
      traps[1].top.x2 = traps[1].top.x1 + pixels_per_row;
      traps[1].top.y  = line.pt1.y;

      traps[1].bot.x2 = itob16(line.pt2.x);
      traps[1].bot.x1 = traps[1].bot.x2 - pixels_per_row;
      traps[1].bot.y  = line.pt2.y;

      if (pixels_per_row < 0)
        {
          b16_t tmp;

          tmp             = traps[1].top.x2;
          traps[1].top.x2 = traps[1].top.x1;
          traps[1].top.x1 = tmp;

          tmp             = traps[1].bot.x2;
          traps[1].bot.x2 = traps[1].bot.x1;
          traps[1].bot.x1 = tmp;
        }

      ginfo("Horizontal traps[1]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
            "(%08" PRIx32 ",%08" PRIx32 ", %d)\n",
            traps[1].top.x1, traps[1].top.x2, traps[1].top.y,
            traps[1].bot.x1, traps[1].bot.x2, traps[1].bot.y);

      return 1;
    }

  /* Okay, then what remains is interesting.
   *
   * iheight = |y2 - y1|
   * iwidth  = |x2 - x1|
   */

  iheight = line.pt2.y - line.pt1.y + 1;
  if (line.pt1.x < line.pt2.x)
    {
      iwidth  = line.pt2.x - line.pt1.x + 1;
    }
  else
    {
      iwidth  = line.pt1.x - line.pt2.x + 1;
    }

  /* Applying the line width to the line results in a rotated, rectangle.
   * Get the Y offset from an end of the original thin line to a corner of
   * the fat line.
   *
   *   Angle of line:      angle      = atan2(iheight, iwidth)
   *   Y offset from line: b16yoffset = linewidth * cos(angle)
   *
   * For near verical lines, b16yoffset is be nearly zero.  For near
   * horizontal lines, b16yOffset is be about the same as linewidth.
   */

  angle      = b16atan2(itob16(iheight), itob16(iwidth));
  cosangle   = b16cos(angle);
  b16yoffset = (linewidth * cosangle + 1) >> 1;

  /* Get the X offset from an end of the original thin line to a corner of
   * the fat line.
   *
   * For near vertical lines, b16xoffset is about the same as linewidth.
   * For near horizontal lines, b16xoffset is nearly zero.
   */

  sinangle   =  b16sin(angle);
  b16xoffset = (linewidth * sinangle + 1) >> 1;

  ginfo("height: %d width: %d angle: %08" PRIx32 " "
        "b16yoffset: %08" PRIx32 " b16xoffset: %08" PRIx32 "\n",
        iheight, iwidth, angle, b16yoffset, b16xoffset);

  /* Now we know all four points of the rotated rectangle */

  iyoffset   = b16toi(b16yoffset + b16HALF);
  if (iyoffset > 0)
    {
      /* Get the Y positions of each point */

      b16y      = itob16(line.pt1.y);
      quad[0].y = b16y - b16yoffset;
      quad[1].y = b16y + b16yoffset;

      b16y      = itob16(line.pt2.y);
      quad[2].y = b16y - b16yoffset;
      quad[3].y = b16y + b16yoffset;

      if (line.pt1.x < line.pt2.x)
        {
          /* Line is going "south east". Get the X positions of each point */

          b16x      = itob16(line.pt1.x);
          quad[0].x = b16x + b16xoffset;
          quad[1].x = b16x - b16xoffset;

          b16x      = itob16(line.pt2.x);
          quad[2].x = b16x + b16xoffset;
          quad[3].x = b16x - b16xoffset;

          ginfo("Southeast: quad (%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 ")\n",
                quad[0].x, quad[0].y, quad[1].x, quad[1].y,
                quad[2].x, quad[2].y, quad[3].x, quad[3].y);

          /* Now we can form the trapezoids.  The top of the first trapezoid
           * (triangle) is at quad[0]
           */

          traps[0].top.x1 = quad[0].x;
          traps[0].top.x2 = quad[0].x;
          traps[0].top.y  = b16toi(quad[0].y + b16HALF);

          /* The bottom of the first trapezoid (triangle) may be either at
           * quad[1] or quad[2], depending upon orientation.
           */

          if (quad[1]. y < quad[2].y)
            {
              /* quad[1] is at the bottom left of the triangle. Interpolate
               * to get the corresponding point on the right side.
               *
               * Interpolation is from quad[0] along the line
               * quad[0]->quad[2] which as the same slope as the line
               * (positive)
               */

              b16dxdy = itob16(iwidth) / iheight;

              traps[0].bot.x1 = quad[1].x;
              traps[0].bot.x2 = nxgl_interpolate(quad[0].x,
                                                 quad[1].y - quad[0].y,
                                                 b16dxdy);
              traps[0].bot.y  = b16toi(quad[1].y + b16HALF);

              /* quad[1] is at the top left of the second trapezoid.
               * quad[2} is at the bottom right of the second trapezoid.
               * Interpolate to get corresponding point on the left side.
               *
               * Interpolation is from quad[1] along the line
               * quad[1]->quad[3] which as the same slope as the line
               * (positive)
               */

              traps[1].top.x1 = traps[0].bot.x1;
              traps[1].top.x2 = traps[0].bot.x2;
              traps[1].top.y  = traps[0].bot.y;

              traps[1].bot.x1 = nxgl_interpolate(traps[1].top.x1,
                                                 quad[2].y - quad[1].y,
                                                 b16dxdy);
              traps[1].bot.x2 = quad[2].x;
              traps[1].bot.y  = b16toi(quad[2].y + b16HALF);
            }
          else
            {
              /* quad[2] is at the bottom right of the triangle. Interpolate
               * to get the corresponding point on the left side.
               *
               * Interpolation is from quad[0] along the line
               * quad[0]->quad[1] which orthogonal to the slope of the line
               * (and negative)
               */

              b16dxdy = -itob16(iheight) / iwidth;

              traps[0].bot.x1 = nxgl_interpolate(quad[0].x,
                                                 quad[2].y - quad[0].y,
                                                 b16dxdy);
              traps[0].bot.x2 = quad[2].x;
              traps[0].bot.y  = b16toi(quad[2].y + b16HALF);

              /* quad[2] is at the top right of the second trapezoid.
               * quad[1} is at the bottom left of the second trapezoid.
               * Interpolate to get corresponding point on the right side.
               *
               * Interpolation is from quad[2] along the line
               * quad[2]->quad[3] which as the same slope as the previous
               * interpolation.
               */

              traps[1].top.x1 = traps[0].bot.x1;
              traps[1].top.x2 = traps[0].bot.x2;
              traps[1].top.y  = traps[0].bot.y;

              traps[1].bot.x1 = quad[1].x;
              traps[1].bot.x2 = nxgl_interpolate(traps[1].top.x2,
                                                 quad[1].y - quad[2].y,
                                                 b16dxdy);
              traps[1].bot.y  = b16toi(quad[1].y + b16HALF);
            }

          /* The final trapezond (triangle) at the bottom is new well
           * defined
           */

          traps[2].top.x1 = traps[1].bot.x1;
          traps[2].top.x2 = traps[1].bot.x2;
          traps[2].top.y  = traps[1].bot.y;

          traps[2].bot.x1 = quad[3].x;
          traps[2].bot.x2 = quad[3].x;
          traps[2].bot.y  = b16toi(quad[3].y + b16HALF);
        }
      else
        {
          /* Get the X positions of each point */

          b16x      = itob16(line.pt1.x);
          quad[0].x = b16x - b16xoffset;
          quad[1].x = b16x + b16xoffset;

          b16x      = itob16(line.pt2.x);
          quad[2].x = b16x - b16xoffset;
          quad[3].x = b16x + b16xoffset;

          ginfo("Southwest: quad (%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 "),"
                "(%08" PRIx32 ",%08" PRIx32 ")\n",
                quad[0].x, quad[0].y, quad[1].x, quad[1].y,
                quad[2].x, quad[2].y, quad[3].x, quad[3].y);

          /* Now we can form the trapezoids.  The top of the first trapezoid
           * (triangle) is at quad[0]
           */

          traps[0].top.x1 = quad[0].x;
          traps[0].top.x2 = quad[0].x;
          traps[0].top.y  = b16toi(quad[0].y + b16HALF);

          /* The bottom of the first trapezoid (triangle) may be either at
           * quad[1] or quad[2], depending upon orientation.
           */

          if (quad[1].y < quad[2].y)
            {
              /* quad[1] is at the bottom right of the triangle. Interpolate
               * to get the corresponding point on the left side.
               *
               * Interpolation is from quad[0] along the line
               * quad[0]->quad[2] which as the same slope as the line
               * (negative)
               */

              b16dxdy = -itob16(iwidth) / iheight;

              traps[0].bot.x1 = nxgl_interpolate(traps[0].top.x1,
                                                 quad[1].y - quad[0].y,
                                                 b16dxdy);
              traps[0].bot.x2 = quad[1].x;
              traps[0].bot.y  = b16toi(quad[1].y + b16HALF);

              /* quad[1] is at the top right of the second trapezoid.
               * quad[2} is at the bottom left of the second trapezoid.
               * Interpolate to get corresponding point on the right side.
               *
               * Interpolation is from quad[1] along the line
               * quad[1]->quad[3] which as the same slope as the line
               * (negative)
               */

              traps[1].top.x1 = traps[0].bot.x1;
              traps[1].top.x2 = traps[0].bot.x2;
              traps[1].top.y  = traps[0].bot.y;

              traps[1].bot.x1 = quad[2].x;
              traps[1].bot.x2 = nxgl_interpolate(traps[1].top.x2,
                                                 quad[2].y - quad[1].y,
                                                 b16dxdy);
              traps[1].bot.y  = b16toi(quad[2].y + b16HALF);
            }
          else
            {
              /* quad[2] is at the bottom left of the triangle. Interpolate
               * to get the corresponding point on the right side.
               *
               * Interpolation is from quad[0] along the line
               * quad[0]->quad[1] which orthogonal to the slope of the line
               * (and positive)
               */

              b16dxdy = itob16(iheight) / iwidth;

              traps[0].bot.x1 = quad[2].x;
              traps[0].bot.x2 = nxgl_interpolate(traps[0].top.x2,
                                                 quad[2].y - quad[0].y,
                                                 b16dxdy);
              traps[0].bot.y  = b16toi(quad[2].y + b16HALF);

              /* quad[2] is at the top left of the second trapezoid.
               * quad[1} is at the bottom right of the second trapezoid.
               * Interpolate to get corresponding point on the left side.
               *
               * Interpolation is from quad[2] along the line
               * quad[2]->quad[3] which as the same slope as the previous
               * interpolation.
               */

              traps[1].top.x1 = traps[0].bot.x1;
              traps[1].top.x2 = traps[0].bot.x2;
              traps[1].top.y  = traps[0].bot.y;

              traps[1].bot.x1 = nxgl_interpolate(traps[1].top.x1,
                                                 quad[1].y - quad[2].y,
                                                 b16dxdy);
              traps[1].bot.x2 = quad[1].x;
              traps[1].bot.y  = b16toi(quad[1].y + b16HALF);
            }

          /* The final trapezond (triangle) at the bottom is new well
           * defined
           */

          traps[2].top.x1 = traps[1].bot.x1;
          traps[2].top.x2 = traps[1].bot.x2;
          traps[2].top.y  = traps[1].bot.y;

          traps[2].bot.x1 = quad[3].x;
          traps[2].bot.x2 = quad[3].x;
          traps[2].bot.y  = b16toi(quad[3].y + b16HALF);
        }

      ginfo("traps[0]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
            "(%08" PRIx32 ",%08" PRIx32 ",%d)\n",
            traps[0].top.x1, traps[0].top.x2, traps[0].top.y,
            traps[0].bot.x1, traps[0].bot.x2, traps[0].bot.y);
      ginfo("traps[1]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
            "(%08" PRIx32 ",%08" PRIx32 ",%d)\n",
            traps[1].top.x1, traps[1].top.x2, traps[1].top.y,
            traps[1].bot.x1, traps[1].bot.x2, traps[1].bot.y);
      ginfo("traps[2]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
            "(%08" PRIx32 ",%08" PRIx32 ",%d)\n",
            traps[2].top.x1, traps[2].top.x2, traps[2].top.y,
            traps[2].bot.x1, traps[2].bot.x2, traps[2].bot.y);

      return 0;
    }

  /* The line is too vertical to have any significant triangular top or
   * bottom.  Just return the center parallelogram.
   */

  traps[1].top.x1 = itob16(line.pt1.x - (linewidth >> 1));
  traps[1].top.x2 = traps[1].top.x1 + itob16(linewidth - 1);
  traps[1].top.y  = line.pt1.y;

  traps[1].bot.x1 = itob16(line.pt2.x - (linewidth >> 1));
  traps[1].bot.x2 = traps[1].bot.x1 + itob16(linewidth - 1);
  traps[1].bot.y  = line.pt2.y;

  ginfo("Horizontal traps[1]: (%08" PRIx32 ",%08" PRIx32 ",%d),"
        "(%08" PRIx32 ",%08" PRIx32 ", %d)\n",
        traps[1].top.x1, traps[1].top.x2, traps[1].top.y,
        traps[1].bot.x1, traps[1].bot.x2, traps[1].bot.y);

  return 1;
}
