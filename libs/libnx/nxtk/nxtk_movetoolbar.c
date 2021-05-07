/****************************************************************************
 * libs/libnx/nxtk/nxtk_movetoolbar.c
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
 * Name: nxtk_movetoolbar
 *
 * Description:
 *   Move a rectangular region within the toolbar sub-window of a framed
 *   window
 *
 * Input Parameters:
 *   hfwnd  - The sub-window containing the toolbar within which the move is
 *            to be done. This must have been previously created by
 *            nxtk_openwindow().
 *   rect   - Describes the rectangular region relative to the toolbar
 *            sub-window to move
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_movetoolbar(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *rect,
                     FAR const struct nxgl_point_s *offset)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_rect_s srcrect;
  struct nxgl_point_s clipoffset;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !rect || !offset)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Make sure that both the source and dest rectangle lie within the
   * toolbar sub-window
   */

  nxtk_subwindowmove(fwnd, &srcrect, &clipoffset,
                     rect, offset, &fwnd->tbrect);

  /* Then move it within the toolbar window */

  return nx_move((NXWINDOW)hfwnd, &srcrect, &clipoffset);
}
