/****************************************************************************
 * graphics/nxbe/nxbe_raise.c
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
