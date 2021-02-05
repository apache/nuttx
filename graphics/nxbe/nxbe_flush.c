/****************************************************************************
 * graphics/nxbe/nxbe_flush.c
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
