/****************************************************************************
 * libs/libnx/nxtk/nxtk_getwindow.c
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
 * Name: nxtk_getwindow
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_getwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                   unsigned int plane, FAR uint8_t *dest,
                   unsigned int deststride)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_rect_s getrect;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !rect || !dest)
    {
      ginfo("Invalid parameters\n");
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Move the rectangle to that it is relative to the containing
   * window. If part of the rectangle lies outside the window,
   * it will contain garbage data, but the contained area will be
   * valid.
   */

  nxgl_rectoffset(&getrect, rect,
                  fwnd->fwrect.pt1.x - fwnd->wnd.bounds.pt1.x,
                  fwnd->fwrect.pt1.y - fwnd->wnd.bounds.pt1.y);

  /* Then get it */

  return nx_getrectangle((NXWINDOW)hfwnd, &getrect, plane, dest, deststride);
}
