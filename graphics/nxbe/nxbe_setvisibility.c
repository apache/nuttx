/****************************************************************************
 * graphics/nxbe/nxbe_setvisibility.c
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
