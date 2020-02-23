/****************************************************************************
 * graphics/nxbe/nxbe_getrectangle.c
 *
 *   Copyright (C) 2011, 2019 Gregory Nutt. All rights reserved.
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

struct nxbe_fill_s
{
  struct nxbe_clipops_s cops;
  nxgl_mxpixel_t color;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_getrectangle_dev
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nxbe_getrectangle_dev(FAR struct nxbe_window_s *wnd,
                                         FAR const struct nxgl_rect_s *rect,
                                         unsigned int plane,
                                         FAR uint8_t *dest,
                                         unsigned int deststride)
{
  FAR struct nxbe_plane_s *pplane = &wnd->be->plane[plane];

  DEBUGASSERT(pplane != NULL && pplane->dev.getrectangle != NULL);

  pplane->dev.getrectangle(&pplane->pinfo, rect, dest, deststride);
}

/****************************************************************************
 * Name: nxbe_getrectangle_pwfb
 *
 * Description:
 *  Get the contents of pre-window framebuffer graphic memory within a
 *  rectangular region.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied (in device coordinates, clipped)
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static inline void nxbe_getrectangle_pwfb(FAR struct nxbe_window_s *wnd,
                                          FAR const struct nxgl_rect_s *rect,
                                          unsigned int plane,
                                          FAR uint8_t *dest,
                                          unsigned int deststride)
{
  FAR struct nxbe_plane_s *pplane = &wnd->be->plane[plane];
  struct nxgl_rect_s relrect;

  DEBUGASSERT(pplane != NULL && pplane->pwfb.getrectangle != NULL);

  /* The rectangle that we receive here is in absolute device coordinates.
   * We need to restore this to windows relative coordinates.
   */

  nxgl_rectoffset(&relrect, rect, -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  /* Then get the rectangle from the framebuffer */

  pplane->pwfb.getrectangle(wnd, &relrect, dest, deststride);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied (in window-relative coordinates)
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_getrectangle(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rect, unsigned int plane,
                       FAR uint8_t *dest, unsigned int deststride)
{
  struct nxgl_rect_s remaining;

  DEBUGASSERT(wnd != NULL && rect != NULL && dest != NULL);
  DEBUGASSERT(wnd->be != NULL && wnd->be->plane != NULL);
  DEBUGASSERT(plane < wnd->be->vinfo.nplanes);

  /* Offset the rectangle by the window origin to convert it into a
   * bounding box in device coordinates
   */

  nxgl_rectoffset(&remaining, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Clip to the bounding box to the limits of the window and of the
   * background screen
   */

  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  /* The return the graphics memory at this location. */

  if (!nxgl_nullrect(&remaining))
    {
#ifdef CONFIG_NX_RAMBACKED
      /* If this window supports a pre-window frame buffer then get the
       * rectangle from pre-window framebuffer.
       */

      if (NXBE_ISRAMBACKED(wnd))
        {
          nxbe_getrectangle_pwfb(wnd, &remaining, plane, dest, deststride);
        }
      else
#endif
      /* If the window is hidden, then there is no available data source */

      if (!NXBE_ISHIDDEN(wnd))
        {
#ifdef CONFIG_NX_SWCURSOR
          /* Is the software cursor visible? */

          if (wnd->be->cursor.visible)
            {
              /* Erase any portion of the cursor that may be above this
               * region.
               * REVISIT:  Only a single color plane is supported
               */

              wnd->be->plane[0].cursor.erase(wnd->be, &remaining, 0);
            }
#endif
          /* Get the rectangle from the graphics device memory.
           * NOTE: Since raw graphic memory is returned, the returned memory
           * content may be the memory of windows above this one and may
           * not necessarily belong to this window.
           */

           nxbe_getrectangle_dev(wnd, &remaining, plane, dest, deststride);

#ifdef CONFIG_NX_SWCURSOR
          /* Was the software cursor visible? */

          if (wnd->be->cursor.visible)
            {
              /* Restore the software cursor if any part of the cursor was
               * erased above.
               */

              wnd->be->plane[0].cursor.draw(wnd->be, &remaining, 0);
            }
#endif
        }
    }
}
