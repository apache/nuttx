/****************************************************************************
 * libs/libnx/nxtk/nxtk_drawlinetoolbar.c
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
#include <nuttx/nx/nxtk.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_drawlinetoolbar
 *
 * Description:
 *  Fill the specified line in the toolbar sub-window with the specified
 *  color.  This is simply a wrapper that uses nxgl_splitline() to break the
 *  line into trapezoids and then calls nxtk_filltrapwindow() to render the
 *  line.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   vector - Describes the line to be drawn
 *   width  - The width of the line
 *   color  - The color to use to fill the line
 *   caps   - Draw a circular cap on the ends of the line to support better
 *            line joins
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawlinetoolbar(NXTKWINDOW hfwnd,
                         FAR struct nxgl_vector_s *vector,
                         nxgl_coord_t width,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES],
                         uint8_t caps)
{
  struct nxgl_trapezoid_s trap[3];
  struct nxgl_rect_s rect;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !vector || width < 1 || !color)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Split the line into trapezoids */

  ret = nxgl_splitline(vector, trap, &rect, width);
  switch (ret)
    {
      /* 0: Line successfully broken up into three trapezoids.
       *  Values in
       *    traps[0], traps[1], and traps[2] are valid.
       */

      case 0:
        ret = nxtk_filltraptoolbar(hfwnd, &trap[0], color);
        if (ret == OK)
          {
            ret = nxtk_filltraptoolbar(hfwnd, &trap[1], color);
            if (ret == OK)
              {
                ret = nxtk_filltraptoolbar(hfwnd, &trap[2], color);
              }
          }
        break;

      /* 1: Line successfully represented by one trapezoid. Value in traps[1]
       *    is valid.
       */

      case 1:
        ret = nxtk_filltraptoolbar(hfwnd, &trap[1], color);
         break;

      /* 2: Line successfully represented by one rectangle. Value in rect is
       *    valid
       */

      case 2:
         ret = nxtk_filltoolbar(hfwnd, &rect, color);
         break;

      /* <0: On errors, a negated errno value is returned. */

      default:
         set_errno(EINVAL);
         return ERROR;
    }

  /* Draw circular caps at each end of the line to support better line
   * joins
   */

  if (caps != NX_LINECAP_NONE && width >= 3)
    {
      nxgl_coord_t radius = width >> 1;

      /* Draw a circle at pt1 */

      ret = OK;
      if ((caps & NX_LINECAP_PT1) != 0)
        {
          ret = nxtk_fillcircletoolbar(hfwnd, &vector->pt1, radius, color);
        }

      /* Draw a circle at pt2 */

      if (ret == OK && (caps & NX_LINECAP_PT2) != 0)
        {
          ret = nxtk_fillcircletoolbar(hfwnd, &vector->pt2, radius, color);
        }
    }

  return ret;
}
