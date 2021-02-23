/****************************************************************************
 * libs/libnx/nxtk/nxtk_setsize.c
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
