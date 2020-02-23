/****************************************************************************
 * graphics/nxbe/nxbe_flush.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxbe.h>

#include "nxbe.h"

#ifdef CONFIG_NX_RAMBACKED

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_flush
 *
 * Description:
 *   After per-window framebuffer has been updated, the modified region must
 *   be written to device graphics memory.  That function is managed by this
 *   simple function.  It does the following:
 *
 *   1) It calls nxbe_bitmap_dev() to copy the modified per-window
 *      framebuffer into device graphics memory.
 *   2) If CONFIG_NX_SWCURSOR is enabled, it calls the cursor "draw"
 *      renderer to update re-draw the currsor image if any portion of
 *      graphics display update overwrote the cursor.  Since these
 *      operations are performed back-to-back, any resulting flicker
 *      should be minimized.
 *
 * Input Parameters (same as for nxbe_flush):
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region in the window that will
 *            receive the the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_flush(FAR struct nxbe_window_s *wnd,
                FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin,
                unsigned int stride)
{
  /* Don't update hidden windows */

  if (!NXBE_ISHIDDEN(wnd))
    {
      /* Copy the modified per-window framebuffer into device memory. */

      nxbe_bitmap_dev(wnd, dest, src, origin, stride);

#ifdef CONFIG_NX_SWCURSOR
      /* Update cursor backup memory and redraw the cursor in the modified
       * window region.
       */

      nxbe_cursor_backupdraw_all(wnd, dest);
#endif
    }
}

#endif /* CONFIG_NX_RAMBACKED */
