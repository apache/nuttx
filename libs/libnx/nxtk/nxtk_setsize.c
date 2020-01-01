/****************************************************************************
 * libs/libnx/nxtk/nxtk_setsize.c
 *
 *   Copyright (C) 2008-2009, 2013, 2019 Gregory Nutt. All rights reserved.
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
 * Name: nxtk_setsize
 *
 * Description:
 *  Set the size for the selected client window.  This size does not
 *  include the sizes of the borders nor for any toolbar.  Those sizes
 *  will be added in to set the full window size.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   size  - The new size of the client sub-window.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_setsize(NXTKWINDOW hfwnd, FAR const struct nxgl_size_s *size)
{
  FAR struct nxtk_framedwindow_s *fwnd;
  struct nxgl_size_s newsize;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (hfwnd == NULL || size == NULL || size->w < 0 || size->h < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  fwnd = (FAR struct nxtk_framedwindow_s *)hfwnd;

  /* Add the sizes need for the toolbar and the borders */

  newsize.w = size->w + 2 * CONFIG_NXTK_BORDERWIDTH;
  newsize.h = size->h + fwnd->tbheight + 2 * CONFIG_NXTK_BORDERWIDTH;

  /* Then set the window size */

  ret = nx_setsize((NXWINDOW)hfwnd, &newsize);

#ifdef CONFIG_NX_RAMBACKED
  /* Windows backed with per-window framebuffers do not get redraw
   * callbacks.  Normally the frame is updated with every redraw callback.
   * However, as a minimum, the frame only has to but updated after the
   * window or toolbar sizes change.
   */

  if (ret >= 0 && NXBE_ISRAMBACKED(&fwnd->wnd))
    {
      struct nxgl_rect_s relbounds;

      /* Convert to a window-relative bounding box */

      nxgl_rectoffset(&relbounds, &fwnd->wnd.bounds,
                      -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);

      /* Then re-draw the frame */

      nxtk_drawframe(fwnd, &relbounds); /* Does not fail */
    }
#endif

  return ret;
}
