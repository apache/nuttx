/****************************************************************************
 * libs/libnx/nxtk/nxtk_filltraptoolbar.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Name: nxtk_filltraptoolbar
 *
 * Description:
 *  Fill the specified rectangle in the toolbar with the specified color
 *
 * Input Parameters:
 *   hfwnd - The handle returned by nxtk_openwindow
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_filltraptoolbar(NXTKWINDOW hfwnd,
                         FAR const struct nxgl_trapezoid_s *trap,
                         nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_rect_s relclip;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !trap || !color)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Perform the fill, clipping to the client window */

  nxgl_rectoffset(&relclip, &fwnd->tbrect, -fwnd->wnd.bounds.pt1.x,
                  -fwnd->wnd.bounds.pt1.y);
  return nx_filltrapezoid((NXWINDOW)hfwnd, &relclip, trap, color);
}
