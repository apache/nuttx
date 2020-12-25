/****************************************************************************
 * graphics/nxbe/nxbe_bitmap.c
 *
 *   Copyright (C) 2008-2009, 2012, 2016, 2019 Gregory Nutt. All rights
 *     reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include "nxbe.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nx_bitmap_s
{
  struct nxbe_clipops_s cops;
  FAR const void *src;              /* The start of the source image. */
  struct nxgl_point_s origin;       /* Offset into the source image data */
  unsigned int stride;              /* The width of the full source image in pixels. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bitmap_clipcopy
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible
 *  portions of the rectangle.
 *
 ****************************************************************************/

static void bitmap_clipcopy(FAR struct nxbe_clipops_s *cops,
                            FAR struct nxbe_plane_s *plane,
                            FAR const struct nxgl_rect_s *rect)
{
  struct nx_bitmap_s *bminfo = (struct nx_bitmap_s *)cops;

  /* Copy the rectangular region to the graphics device. */

  plane->dev.copyrectangle(&plane->pinfo, rect, bminfo->src,
                           &bminfo->origin, bminfo->stride);

#ifdef CONFIG_NX_UPDATE
  /* Notify external logic that the display has been updated */

  nxbe_notify_rectangle(plane->driver, rect);
#endif
}

/****************************************************************************
 * Name: nxbe_bitmap_pwfb
 *
 * Description:
 *   Copy a rectangular region of a larger image into the per-window
 *   framebuffer.
 *
 * Input Parameters:
 *   wnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular on the display that will receive the
 *            the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static inline void nxbe_bitmap_pwfb(FAR struct nxbe_window_s *wnd,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src[CONFIG_NX_NPLANES],
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int stride)
{
  struct nxgl_rect_s destrect;
  unsigned int deststride;

  DEBUGASSERT(wnd != NULL && dest != NULL && src != NULL && origin != NULL);
  DEBUGASSERT(wnd->be != NULL && wnd->be->plane != NULL);

  /* Verify that the destination rectangle begins "below" and to the "right"
   * of the origin
   */

  if (dest->pt1.x < origin->x || dest->pt1.y < origin->y)
    {
      gerr("ERROR: Bad dest start position\n");
      return;
    }

  /* Verify that the width of the destination rectangle does not exceed the
   * width of the source bitmap data (taking into account the bitmap origin)
   */

  deststride = (((dest->pt2.x - origin->x + 1) *
                 wnd->be->plane[0].pinfo.bpp + 7) >> 3);
  if (deststride > stride)
    {
      gerr("ERROR: Bad dest width\n");
      return;
    }

  /* Offset the rectangle and image origin by the window origin.  This
   * converts the rectangle in the device absolute coordinates.
   */

  nxgl_rectoffset(&destrect, dest, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Clip to the limits of the window and of the background screen (in
   * device coordinates.
   */

  nxgl_rectintersect(&destrect, &destrect, &wnd->bounds);
  nxgl_rectintersect(&destrect, &destrect, &wnd->be->bkgd.bounds);

  if (!nxgl_nullrect(&destrect))
    {
      /* Restore the destination rectangle to relative window coordinates. */

      nxgl_rectoffset(&destrect, &destrect,
                      -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

      /* Copy the rectangular region to the framebuffer (no clipping).
       * REVISIT:  Assumes a single color plane.
       */

      DEBUGASSERT(wnd->be->plane[0].pwfb.copyrectangle != NULL);
      wnd->be->plane[0].pwfb.copyrectangle(wnd, &destrect, src[0],
                                           origin, stride);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_bitmap_dev
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  The graphics output is written to the graphics
 *   device unconditionally.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

void nxbe_bitmap_dev(FAR struct nxbe_window_s *wnd,
                     FAR const struct nxgl_rect_s *dest,
                     FAR const void *src[CONFIG_NX_NPLANES],
                     FAR const struct nxgl_point_s *origin,
                     unsigned int stride)
{
  struct nx_bitmap_s info;
  struct nxgl_rect_s bounds;
  struct nxgl_point_s offset;
  struct nxgl_rect_s remaining;
  unsigned int deststride;
  int i;

  DEBUGASSERT(wnd != NULL && dest != NULL && src != NULL && origin != NULL);
  DEBUGASSERT(wnd->be != NULL && wnd->be->plane != NULL);

  /* Don't update hidden windows */

  if (NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  /* Verify that the destination rectangle begins "below" and to the "right"
   * of the origin
   */

  if (dest->pt1.x < origin->x || dest->pt1.y < origin->y)
    {
      gerr("ERROR: Bad dest start position\n");
      return;
    }

  /* Verify that the width of the destination rectangle does not exceed the
   * width of the source bitmap data (taking into account the bitmap origin)
   */

  deststride = (((dest->pt2.x - origin->x + 1) *
                 wnd->be->plane[0].pinfo.bpp + 7) >> 3);
  if (deststride > stride)
    {
      gerr("ERROR: Bad dest width\n");
      return;
    }

  /* Offset the rectangle and image origin by the window origin */

  nxgl_rectoffset(&bounds, dest, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
  nxgl_vectoradd(&offset, origin, &wnd->bounds.pt1);

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, &bounds, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  if (nxgl_nullrect(&remaining))
    {
      return;
    }

  /* Then perform the clipped fill */

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      DEBUGASSERT(wnd->be->plane[i].dev.copyrectangle != NULL);

      info.cops.visible  = bitmap_clipcopy;
      info.cops.obscured = nxbe_clipnull;
      info.src           = src[i];
      info.origin.x      = offset.x;
      info.origin.y      = offset.y;
      info.stride        = stride;

      nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                   &info.cops, &wnd->be->plane[i]);
    }
}

/****************************************************************************
 * Name: nxbe_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  This is a front end to nxbe_bitmap_dev() that is
 *   used only if CONFIG_NX_RAMBACKED=y.  If the per-window frame buffer is
 *   selected, then the bit map will be written to both the graphics device
 *   and shadowed in the per-window framebuffer.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
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

void nxbe_bitmap(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *dest,
                 FAR const void *src[CONFIG_NX_NPLANES],
                 FAR const struct nxgl_point_s *origin,
                 unsigned int stride)
{
#ifdef CONFIG_NX_RAMBACKED
  /* If this window supports a pre-window frame buffer then shadow the full,
   * unclipped bitmap in that framebuffer.
   */

  if (NXBE_ISRAMBACKED(wnd))
    {
      /* Update the per-window framebuffer */

      nxbe_bitmap_pwfb(wnd, dest, src, origin, stride);
    }
#endif

  /* Don't update hidden windows */

  if (!NXBE_ISHIDDEN(wnd))
    {
      /* Rend the bitmap directly to the graphics device */

      nxbe_bitmap_dev(wnd, dest, src, origin, stride);

#ifdef CONFIG_NX_SWCURSOR
      /* Update cursor backup memory and redraw the cursor in the modified
       * window region.
       */

      nxbe_cursor_backupdraw_all(wnd, dest);
#endif
    }
}
