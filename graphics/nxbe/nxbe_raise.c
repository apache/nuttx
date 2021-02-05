/****************************************************************************
 * graphics/nxbe/nxbe_raise.c
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
 * Private Types
 ****************************************************************************/

struct nxbe_raise_s
{
  struct nxbe_clipops_s cops;
  FAR struct nxbe_window_s *wnd;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 ****************************************************************************/

void nxbe_raise(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s *be = wnd->be;

  /* A modal window should already be at the top of the hierarchy. */

  DEBUGASSERT(!NXBE_ISMODAL(wnd) || wnd->above == NULL);

  /* If this window is already at the top of the display, then do nothing
   * (this covers modal window which must always be at the top).  Don't
   * raise the background window and don't raise hidden windows.
   */

  if (wnd->above == NULL || wnd->below == NULL || NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  /* This is some non-modal, window above the background.  If we are in a
   * modal state (i.e., there is some other modal window at the top of the
   * heirary), and it is already as high as it can go in the hierarchy, then
   * do nothing.
   */

  if (NXBE_STATE_ISMODAL(be) && be->topwnd->below == wnd)
    {
      return;
    }

  /* Remove window from the list.  Note that there is always
   * some below this window (it may only be the background window)
   */

  wnd->above->below  = wnd->below;
  wnd->below->above  = wnd->above;

  /* Then put it back in the list. If the top window is a modal window, then
   * only raise it to second highest.
   */

  if (NXBE_STATE_ISMODAL(be) && be->topwnd->below != NULL)
    {
      FAR struct nxbe_window_s *above;
      FAR struct nxbe_window_s *below;

      /* We are in a modal state.  The topwnd is not the background and it
       * has focus.
       */

      above             = be->topwnd;
      below             = be->topwnd->below;

      wnd->above        = above;
      wnd->below        = below;

      above->below      = wnd;
      below->above      = wnd;

      /* Then redraw this window AND all windows below it. Having moved the
       * window, we may have exposed previously obscured portions of windows
       * below this one.
       */

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
       * that it is not obscured by another window
       */

      nxmu_redraw(wnd, &wnd->bounds);
    }
}
