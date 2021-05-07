/****************************************************************************
 * libs/libnx/nxtk/nxtk_drawframe.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxtk.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_drawframeside
 ****************************************************************************/

#if CONFIG_NXTK_BORDERWIDTH > 0
static void nxtk_drawframeside(FAR struct nxtk_framedwindow_s *fwnd,
                               FAR const struct nxgl_rect_s *side,
                               FAR const struct nxgl_rect_s *bounds,
                               nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxgl_rect_s intersection;
  nxgl_rectintersect(&intersection, side, bounds);
  if (!nxgl_nullrect(&intersection))
    {
      nx_fill((NXWINDOW)fwnd, &intersection, color);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_drawframe
 *
 * Description:
 *   Redraw the window frame.
 *
 * Input Parameters:
 *   fwnd   - the framed window whose frame needs to be re-drawn.  This must
 *            have been previously created by nxtk_openwindow().
 *   bounds - Only draw the parts of the frame within this bounding box.
 *            (window relative coordinates).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_drawframe(FAR struct nxtk_framedwindow_s *fwnd,
                   FAR const struct nxgl_rect_s *bounds)
{
#if CONFIG_NXTK_BORDERWIDTH > 0
  struct nxgl_rect_s frame;
#endif
  struct nxgl_size_s wndsize;
  struct nxgl_size_s tbsize;
#if CONFIG_NXTK_BORDERWIDTH > 0
  nxgl_coord_t thickness;
#endif

  /* Shiny edge:
   *   Thickness: 1
   *   Color:     CONFIG_NXTK_BORDERCOLOR3;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 2
   * Central part:
   *   Thickness: Varies with CONFIG_NXTK_BORDERWIDTH
   *   Color:     CONFIG_NXTK_BORDERCOLOR1;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 0
   * Shadow part:
   *   Thickness: 1;
   *   Color:     CONFIG_NXTK_BORDERCOLOR2;
   *   Condition: CONFIG_NXTK_BORDERWIDTH > 1
   */

#if CONFIG_NXTK_BORDERWIDTH > 2
  thickness = CONFIG_NXTK_BORDERWIDTH - 2;
#elif CONFIG_NXTK_BORDERWIDTH > 1
  thickness = CONFIG_NXTK_BORDERWIDTH - 1;
#elif CONFIG_NXTK_BORDERWIDTH > 0
  thickness = CONFIG_NXTK_BORDERWIDTH;
#endif

  /* Get the size of the rectangle */

  nxgl_rectsize(&wndsize, &fwnd->wnd.bounds);
  nxgl_rectsize(&tbsize, &fwnd->tbrect);

  /* Draw the top ***********************************************************/

#if CONFIG_NXTK_BORDERWIDTH > 0
  frame.pt1.x = 0;
  frame.pt2.x = wndsize.w - 1;
  frame.pt1.y = 0;

  /* Draw the shiny edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.y = 0;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
  frame.pt1.y = 1;
#endif

  /* Draw the central part */

  frame.pt2.y = frame.pt1.y + thickness - 1;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Draw a single line under the toolbar, color CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y += tbsize.h + thickness;
  frame.pt2.y  = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw the bottom ********************************************************/

#if CONFIG_NXTK_BORDERWIDTH > 0
  frame.pt1.y = wndsize.h - CONFIG_NXTK_BORDERWIDTH;

  /* Draw the shiny edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.y = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
  frame.pt1.y ++;
#endif

  /* Draw the central part */

  frame.pt2.y = frame.pt1.y + thickness - 1;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

  /* Then a single line at the very bottom, Color: CONFIG_NXTK_BORDERCOLOR2 */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.y = wndsize.h - 1;
  frame.pt2.y = frame.pt1.y;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif
#endif

  /* Draw left and right outer edges ****************************************/

  /* Draw the shiny left out edge */

#if CONFIG_NXTK_BORDERWIDTH > 1
  frame.pt1.x = 0;
  frame.pt1.y = 1;
#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - 2;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
#endif

  /* Draw the shadowed right outer edge */

  frame.pt1.x = wndsize.w - 1;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);
#endif

  /* Draw left and right central regions ************************************/

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = 1;
  frame.pt1.y = 1;
  frame.pt2.x = frame.pt1.x + thickness - 1;
  frame.pt2.y = wndsize.h - 2;
#else
  frame.pt1.x = 0;
  frame.pt1.y = 0;
  frame.pt2.x = frame.pt1.x + thickness - 1;
  frame.pt2.y = wndsize.h - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = wndsize.w - thickness - 1;
  frame.pt2.x = wndsize.w - 2;
#else
  frame.pt1.x = wndsize.w - thickness;
  frame.pt2.x = wndsize.w - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);
#endif

  /* Draw left and right inner sides ****************************************/

  /* This segment stops at the bottom of the toolbar.  If there is a
   * tool bar, then we have to continue this to the top of the display
   * using g_bordercolor1 (see below)
   */

  /* Draw the shadowed left inner edge */

#if CONFIG_NXTK_BORDERWIDTH > 1
#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = thickness + 1;
  frame.pt1.y = tbsize.h + thickness + 1;
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - thickness - 2;
#else
  frame.pt1.x = thickness;
  frame.pt1.y = tbsize.h + thickness;
  frame.pt2.x = frame.pt1.x;
  frame.pt2.y = wndsize.h - thickness - 1;
#endif
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor2);

  /* Draw the shiny right inner edge */

#if CONFIG_NXTK_BORDERWIDTH > 2
  frame.pt1.x = wndsize.w - thickness - 2;
  frame.pt2.x = frame.pt1.x;
  nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor3);
#endif
#endif

  /* Fix up the little line-seqments at the top inner edges that need to
   * match the color of the toolbar.
   */

#if CONFIG_NXTK_BORDERWIDTH > 1
  if (tbsize.h > 0)
    {
      /* Draw the right side */

#if CONFIG_NXTK_BORDERWIDTH > 2
      frame.pt1.x = thickness + 1;
      frame.pt1.y = 1;
      frame.pt2.x = frame.pt1.x;
      frame.pt2.y = tbsize.h + thickness;
#else
      frame.pt1.x = thickness;
      frame.pt1.y = 0;
      frame.pt2.x = frame.pt1.x;
      frame.pt2.y = tbsize.h + thickness - 1;
#endif
      nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);

      /* Draw the left size */

#if CONFIG_NXTK_BORDERWIDTH > 2
      frame.pt1.x = wndsize.w - thickness - 2;
      frame.pt2.x = frame.pt1.x;
      nxtk_drawframeside(fwnd, &frame, bounds, g_bordercolor1);
#endif
    }
#endif

  return OK;
}
