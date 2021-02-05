/****************************************************************************
 * graphics/nxbe/nxbe_lower.c
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

#include <stddef.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include "nxbe.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 ****************************************************************************/

void nxbe_lower(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s  *be = wnd->be;
  FAR struct nxbe_window_s *below;

  /* If the window is already at the bottom, then there is nothing to do.
   * Refuse to lower the background window; Refuse to lower a modal window.
   * It is impossible to lower a hidden window because it does not exist
   * in the hierarchy.
   */

  if (wnd->below == NULL || wnd->below == &be->bkgd ||
      NXBE_ISMODAL(wnd) || NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  /* Remove the window from its current position in the list */

  wnd->below->above = wnd->above;

  /* Was it at the top of the display? */

  if (wnd->above)
    {
      /* No... it was in the middle somewhere */

      wnd->above->below = wnd->below;
    }
  else
    {
      /* Yes.. set the new top window */

      be->topwnd        = wnd->below;
      be->topwnd->above = NULL;
    }

  /* Remember the window that was just below us */

  below = wnd->below;

  /* Then put the lowered window at the bottom (just above the background
   * window).
   */

  wnd->below     = &be->bkgd;
  wnd->above     = be->bkgd.above;
  be->bkgd.above = wnd;

  /* Redraw the windows that were below us (but now are above) */

  nxbe_redrawbelow(be, below, &wnd->bounds);
}
