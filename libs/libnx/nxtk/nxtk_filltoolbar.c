/****************************************************************************
 * libs/libnx/nxtk/nxtk_filltoolbar.c
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
 * Name: nxtk_filltoolbar
 *
 * Description:
 *  Fill the specified rectangle in the client window with the specified
 *  color
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *   rect  - The location within the toolbar window to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxtk_framedwindow_s *fwnd =
                              (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_rect_s fillrect;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !rect || !color)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Clip the rectangle so that it lies within the sub-window bounds
   * then move the rectangle to that it is relative to the containing
   * window.
   */

  nxtk_subwindowclip(fwnd, &fillrect, rect, &fwnd->tbrect);

  /* Then fill it */

  return nx_fill((NXWINDOW)hfwnd, &fillrect, color);
}
