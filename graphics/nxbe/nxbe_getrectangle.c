/****************************************************************************
 * graphics/nxbe/nxbe_getrectangle.c
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
                       FAR const struct nxgl_rect_s *rect,
                       unsigned int plane,
                       FAR uint8_t *dest,
                       unsigned int deststride)
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
