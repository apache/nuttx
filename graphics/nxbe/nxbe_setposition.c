/****************************************************************************
 * graphics/nxbe/nxbe_setposition.c
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
//#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Descripton:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_point_s *pos)
{
  struct nxgl_rect_s rect;

#ifdef CONFIG_DEBUG
  if (!wnd)
    {
      return;
    }
#endif

  /* Back out the old window origin position from the bounding box */

  nxgl_rectoffset(&rect, &wnd->bounds, -wnd->origin.x, -wnd->origin.y);

  /* Set the new origin */

  wnd->origin.x = pos->x;
  wnd->origin.y = pos->y;

  /* Add the new window origin back into the bounding box */

  nxgl_rectoffset(&wnd->bounds, &rect, wnd->origin.x, wnd->origin.y);

  /* Clip the rectangle so that is lies with the screen defined by the
   * background window.
   */

  nxgl_rectintersect(&rect, &rect, &wnd->be->bkgd.bounds);

  /* Then redraw this window AND all windows below it. Having moved the
   * window, we may have exposed previoulsy obscured portions of windows
   * below this one.
   */

  nxbe_redrawbelow(wnd->be, wnd, &rect);

  /* Report the new size/position */

  nxfe_reportposition(wnd);
}
