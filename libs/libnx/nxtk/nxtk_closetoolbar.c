/****************************************************************************
 * libs/libnx/nxtk/nxtk_closetoolbar.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_closetoolbar
 *
 * Description:
 *   Create a tool bar at the top of the specified framed window
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_closetoolbar(NXTKWINDOW hfwnd)
{
  FAR struct nxtk_framedwindow_s *fwnd =
    (FAR struct nxtk_framedwindow_s *)hfwnd;

  /* Un-initialize the toolbar info */

  fwnd->tbheight = 0;
  fwnd->tbcb     = NULL;
  fwnd->tbarg    = NULL;

  /* Calculate the new dimensions of the client window */

  nxtk_setsubwindows(fwnd);

#ifdef CONFIG_NX_RAMBACKED
  /* The redraw request has no effect if a framebuffer is used with the
   * window.  For that type of window, the application must perform the
   * window update itself and not rely on a redraw notification.
   */

  if (NXBE_ISRAMBACKED(&fwnd->wnd))
    {
      struct nxgl_rect_s relbounds;

      /* Convert to a window-relative bounding box */

      nxgl_rectoffset(&relbounds, &fwnd->wnd.bounds,
                      -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);

      /* Then re-draw the frame */

      nxtk_drawframe(fwnd, &relbounds); /* Does not fail */
    }
  else
#endif
    {
      /* Redraw the entire window, even the client window must be redrawn
       * because it has changed its vertical position and size.
       */

      nx_redrawreq(&fwnd->wnd, &fwnd->wnd.bounds);
    }

  return OK;
}
