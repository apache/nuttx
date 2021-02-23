/****************************************************************************
 * libs/libnx/nxtk/nxtk_setsubwindows.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_setsubwindows
 *
 * Description:
 *   Give the window dimensions, border width, and toolbar height,
 *   calculate the new dimensions of the toolbar region and client window
 *   region
 *
 ****************************************************************************/

void nxtk_setsubwindows(FAR struct nxtk_framedwindow_s *fwnd)
{
  nxgl_coord_t fullheight;
  nxgl_coord_t bdrheight = 0;
  nxgl_coord_t tbtop     = fwnd->wnd.bounds.pt1.y;
  nxgl_coord_t tbheight  = 0;
  nxgl_coord_t fwtop     = fwnd->wnd.bounds.pt1.y;
  nxgl_coord_t fwheight  = 0;
  nxgl_coord_t fullwidth;
  nxgl_coord_t bdrwidth;
  nxgl_coord_t fwwidth;
  nxgl_coord_t fwleft;

  /* Divide up the vertical dimension of the window */

  fullheight = fwnd->wnd.bounds.pt2.y - fwnd->wnd.bounds.pt1.y + 1;

  /* Is it tall enough for a border? */

  if (fullheight > 0)
    {
      /* Get the border height */

      bdrheight = ngl_min(2 * CONFIG_NXTK_BORDERWIDTH, fullheight);

      /* Position the toolbar and client window just under the top border */

#if CONFIG_NXTK_BORDERWIDTH > 1
      tbtop += CONFIG_NXTK_BORDERWIDTH - 1;
      fwtop = tbtop + 1;
#else
      tbtop += CONFIG_NXTK_BORDERWIDTH;
      fwtop = tbtop;
#endif

      /* Is it big enough for any part of the toolbar? */

      if (fullheight > 2 * CONFIG_NXTK_BORDERWIDTH)
        {
          /* Yes.. get the height of the toolbar */

          tbheight  = fwnd->tbheight;
          if (tbheight >= fullheight - bdrheight)
            {
              tbheight = fullheight - bdrheight;
            }
          else
            {
              /* And the client window gets whatever is left */

              fwheight = fullheight - bdrheight - tbheight;
            }

          /* Position the client window just under the toolbar */

          fwtop += tbheight;
        }
    }

  /* Divide up the horizontal dimensions of the window */

  fullwidth = fwnd->wnd.bounds.pt2.x - fwnd->wnd.bounds.pt1.x + 1;
  bdrwidth  = ngl_min(2 * CONFIG_NXTK_BORDERWIDTH, fullwidth);
  fwwidth   = fullwidth - bdrwidth;
  fwleft    = fwnd->wnd.bounds.pt1.x + bdrwidth / 2;

  /* Realize the positions/dimensions */

  fwnd->tbrect.pt1.x = fwleft;
  fwnd->tbrect.pt1.y = tbtop;
  fwnd->tbrect.pt2.x = fwleft + fwwidth - 1;
  fwnd->tbrect.pt2.y = tbtop + tbheight - 1;

  fwnd->fwrect.pt1.x = fwleft;
  fwnd->fwrect.pt1.y = fwtop;
  fwnd->fwrect.pt2.x = fwleft + fwwidth - 1;
  fwnd->fwrect.pt2.y = fwtop + fwheight - 1;
}
