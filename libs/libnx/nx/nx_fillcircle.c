/****************************************************************************
 * libs/libnx/nx/nx_fillcircle.c
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

#include <sys/types.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NCIRCLE_TRAPS 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_fillcircle
 *
 * Description:
 *  Fill a circular region using the specified color.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   color  - The color to use to fill the circle.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_fillcircle(NXWINDOW hwnd,
                  FAR const struct nxgl_point_s *center,
                  nxgl_coord_t radius,
                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxgl_trapezoid_s traps[NCIRCLE_TRAPS];
  int i;
  int ret;

  /* Describe the circular region as a sequence of 8 trapezoids */

  nxgl_circletraps(center, radius, traps);

  /* Then rend those trapezoids */

  for (i = 0; i < NCIRCLE_TRAPS; i++)
    {
      ret = nx_filltrapezoid(hwnd, NULL, &traps[i], color);
      if (ret != OK)
        {
          return ret;
        }
    }

  return OK;
}
