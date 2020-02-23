/****************************************************************************
 * graphics/nxbe/nxbe_setvisibility.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_show_window
 *
 * Description:
 *   Make a hidden window visible.
 *
 * Input Parameters:
 *   wnd  - The window to be shown
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_show_window(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s *be = wnd->be;

  /* Mark the window no longer hidden */

  NXBE_CLRHIDDEN(wnd);

  /* Restore the window to the top of the hierarchy.  Exception:  If the top
   * window is a modal window, then only raise it to second highest.
   */

  if (NXBE_STATE_ISMODAL(be) && be->topwnd->below != NULL)
    {
      /* We are in a modal state.  The topwnd is not the background and it
       * has focus.
       */

      wnd->above        = be->topwnd;
      wnd->below        = be->topwnd->below;

      be->topwnd->below = wnd;

     /* Redraw this window and the other that are below us */

      nxbe_redrawbelow(be, wnd, &wnd->bounds);
    }
  else
    {
      /* Otherwise re-insert the window at the top on the display. */

      wnd->above        = NULL;
      wnd->below        = be->topwnd;

      be->topwnd->above = wnd;
      be->topwnd        = wnd;

      /* This window is now at the top of the display, we know, therefore,
       * that it is not obscured by another window.  Just redraw it.
       */

      nxmu_redraw(wnd, &wnd->bounds);
    }
}

/****************************************************************************
 * Name: nxbe_hide_window
 *
 * Description:
 *   Hide a visible window.
 *
 * Input Parameters:
 *   wnd  - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_hide_window(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s *be = wnd->be;

  /* The background window should never be hidden */

  DEBUGASSERT(wnd != &be->bkgd);

  /* Remove this window from the hiearachy */

  /* Is there a window above the one being hidden? */

  if (wnd->above != NULL)
    {
      /* Yes, now the window below that one is the window below
       * the one being hidden.
       */

      wnd->above->below = wnd->below;
    }
  else
    {
      /* No, then the top window is the one below this (which
       * can never be NULL because the background window is
       * always at the true bottom of the list
       */

      be->topwnd = wnd->below;
    }

  /* There is always a window below the one being closed (because
   * the background is never closed.  Now, the window above that
   * is the window above the one that is being closed.
   */

  wnd->below->above = wnd->above;

  /* Redraw the windows that were below us (and may now be exposed) */

  nxbe_redrawbelow(be, wnd->below, &wnd->bounds);

  /* And mark the window as hidden */

  NXBE_SETHIDDEN(wnd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present will will update normally, but will be on the visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   wnd  - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setvisibility(FAR struct nxbe_window_s *wnd, bool hide)
{
  /* Are we showing the window?  Or hiding it? */

  if (hide && !NXBE_ISHIDDEN(wnd))
    {
      nxbe_hide_window(wnd);
    }
  else if (!hide && NXBE_ISHIDDEN(wnd))
    {
      nxbe_show_window(wnd);
    }
}
