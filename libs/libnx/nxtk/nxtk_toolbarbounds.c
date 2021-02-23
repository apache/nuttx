/****************************************************************************
 * libs/libnx/nxtk/nxtk_toolbarbounds.c
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
#include <assert.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_toolbarbounds
 *
 * Description:
 *   Return a bounding box that contains the toolbar in the coordinates of
 *   the containing, framed window.  For example, the recturned  origin
 *  (rect.pt1) is the offset toolbar in the framed window.
 *
 *   NOTE: This function is unsafe in the case of the multi-user NX server
 *   where the width of the window may be being changed asynchronously!  It
 *   may return the old size in this case.
 *
 * Input Parameters:
 *   hfwnd  - The handle returned by nxtk_openwindow
 *   bounds - User provided location in which to return the bounding box.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_toolbarbounds(NXTKWINDOW hfwnd, FAR struct nxgl_rect_s *bounds)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hfwnd;

  DEBUGASSERT(hfwnd && bounds);

  /* Offset the rectangle by subtracting the current position of the
   * window.
   */

  nxgl_rectoffset(bounds, &fwnd->tbrect,
                  -fwnd->wnd.bounds.pt1.x, -fwnd->wnd.bounds.pt1.y);
  return OK;
}
