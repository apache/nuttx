/****************************************************************************
 * libs/libnx/nxtk/nxtk_closetoolbar.c
 *
 *   Copyright (C) 2008-2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
