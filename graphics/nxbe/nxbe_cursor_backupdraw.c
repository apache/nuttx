/****************************************************************************
 * graphics/nxbe/nxbe_cursor_backupdraw.c
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

#include "nxglib.h"
#include "nxbe.h"

#ifdef CONFIG_NX_SWCURSOR

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _nxbe_cursor_backupdraw_dev
 *
 * Description:
 *   Called after any modification to the display to backup and redraw one
 *   color plane
 *
 * Input Parameters:
 *   be    - The back-end state structure instance
 *   rect  - The modified region of the display, in device coordinates
 *   plane - The plane number to use.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void _nxbe_cursor_backupdraw_dev(FAR struct nxbe_state_s *be,
                                               FAR const struct nxgl_rect_s *rect,
                                               int plane)
{
  /* Save the modified cursor background region. */

  be->plane[plane].cursor.backup(be, rect, plane);

  /* Restore the software cursor in the region that was modified. */

  be->plane[plane].cursor.draw(be, rect, plane);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_cursor_backupdraw
 *
 * Description:
 *   Called after any modification to the display (in window coordinate
 *   frame) to perform the backup-draw operation on one color plane.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window, in windows coordinates
 *   plane - The plane number to use.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw_dev(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *rect,
                                int plane)
{
  struct nxgl_rect_s bounds;

  /* Update the software cursor if it is visible */

  if (be->cursor.visible)
    {
      /* Clip to the limits of the display */

      nxgl_rectintersect(&bounds, rect, &be->bkgd.bounds);
      if (!nxgl_nullrect(&bounds))
        {
          _nxbe_cursor_backupdraw_dev(be, &bounds, plane);
        }
    }
}

void nxbe_cursor_backupdraw(FAR struct nxbe_window_s *wnd,
                            FAR const struct nxgl_rect_s *rect,
                            int plane)
{
  struct nxgl_rect_s bounds;

  /* Update the software cursor if it is visible */

  if (wnd->be->cursor.visible)
    {
      /* Offset the rectangle to convert to device coordinates */

      nxgl_rectoffset(&bounds, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

      /* Clip to the limits of the window */

      nxgl_rectintersect(&bounds, &bounds, &wnd->bounds);

      /* Let nxbe_cursor_backupdraw_dev() do the rest */

      nxbe_cursor_backupdraw_dev(wnd->be, &bounds, plane);
    }
}

/****************************************************************************
 * Name: nxbe_cursor_backupdraw_all and nxbe_cursor_backupdraw_devall
 *
 * Description:
 *   Called after any modification to the display to perform the backup-draw
 *   operation on all color planes.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window.  In windows coordinates for
 *           nxbe_cursor_backupdraw(); in graphics device coordinates for
 *           nxbe_cursor_backupdraw_dev().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw_devall(FAR struct nxbe_state_s *be,
                                   FAR const struct nxgl_rect_s *rect)
{
#if CONFIG_NX_NPLANES > 1
  struct nxgl_rect_s bounds;
  int plane;

  /* Update the software cursor if it is visible */

  if (be->cursor.visible)
    {
      /* Clip to the limits of the display */

      nxgl_rectintersect(&bounds, &bounds, &be->bkgd.bounds);
      if (!nxgl_nullrect(&bounds))
        {
          /* Perform the backup-draw operation on all color planes */

          for (plane = 0; plane < CONFIG_NX_NPLANES; plane++)
            {
              _nxbe_cursor_backupdraw_dev(be, &bounds, plane);
            }
        }
    }
#else
  nxbe_cursor_backupdraw_dev(be, rect, 0);
#endif
}

void nxbe_cursor_backupdraw_all(FAR struct nxbe_window_s *wnd,
                                FAR const struct nxgl_rect_s *rect)
{
#if CONFIG_NX_NPLANES > 1
  struct nxgl_rect_s bounds;

  /* Update the software cursor if it is visible */

  if (wnd->be->cursor.visible)
    {
      /* Offset the rectangle to convert it to device coordinates */

      nxgl_rectoffset(&bounds, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

      /* Clip to the limits of the window */

      nxgl_rectintersect(&bounds, &bounds, &wnd->bounds);

      /* And then let nxbe_cursor_backupdraw_devall() do the rest */

      nxbe_cursor_backupdraw_all(wnd, &bounds);
    }
#else
  nxbe_cursor_backupdraw(wnd, rect, 0);
#endif
}

#endif /* CONFIG_NX_SWCURSOR */
