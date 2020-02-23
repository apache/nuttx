/****************************************************************************
 * graphics/nxbe/nxbe_lower.c
 *
 *   Copyright (C) 2008-2009, 2011, 2019 Gregory Nutt. All rights reserved.
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
