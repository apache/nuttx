/****************************************************************************
 * graphics/nxbe/nxbe_raise.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>

#include <nuttx/nxglib.h>

#include "nxbe.h"
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_raise_s
{
  struct nxbe_clipops_s cops;
  FAR struct nxbe_window_s *wnd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipraise
 ****************************************************************************/

static void nxbe_clipraise(FAR struct nxbe_clipops_s *cops,
                          FAR struct nxbe_plane_s *plane,
                          FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *wnd = ((struct nxbe_raise_s *)cops)->wnd;
  nxfe_redrawreq(wnd, rect);
}

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
  struct nxgl_rect_s       rect;
  struct nxbe_raise_s        info;

  if (!wnd->above)
    {
      return;
    }

  /* Redraw the bits that are currently obscured */

  nxgl_rectintersect(&rect, &wnd->bounds, &be->bkgd.bounds);
  if (!nxgl_nullrect(&rect))
    {
      int i;

      info.cops.visible  = nxbe_clipnull;
      info.cops.obscured = nxbe_clipraise;
      info.wnd           = wnd;

#if CONFIG_NX_NPLANES > 1
      for (i = 0; i < be->vinfo.nplanes; i++)
#else
      i = 0;
#endif
        {
          nxbe_clipper(wnd->above, &rect, NX_CLIPORDER_DEFAULT,
                       &info.cops, &be->plane[i]);
        }
    }

  /* Remove window from the list.  Note that there is always
   * some below this window (it may only be the background window)
   */

  wnd->above->below  = wnd->below;
  wnd->below->above  = wnd->above;

  /* Then put it back in the list at the top */

  wnd->above         = NULL;
  wnd->below         = be->topwnd;

  be->topwnd->above  = wnd;
  be->topwnd         = wnd;
}
