/****************************************************************************
 * graphics/nxbe/nxbe_redraw.c
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

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_redraw_s
{
  struct nxbe_clipops_s cops;
  FAR struct nxbe_window_s *wnd;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipredraw
 ****************************************************************************/

static void nxbe_clipredraw(FAR struct nxbe_clipops_s *cops,
                            FAR struct nxbe_plane_s *plane,
                            FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *wnd = ((struct nxbe_redraw_s *)cops)->wnd;
  if (wnd)
    {
      nxmu_redraw(wnd, rect);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_redraw
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for the
 *   specified window
 *
 ****************************************************************************/

void nxbe_redraw(FAR struct nxbe_state_s *be,
                 FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_redraw_s info;
  struct nxgl_rect_s remaining;
#if CONFIG_NX_NPLANES > 1
  int i;
#endif

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, rect, &be->bkgd.bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  if (!nxgl_nullrect(&remaining))
    {
      /* Now, request to re-draw any visible rectangular regions not obscured
       * by windows above this one.
       */

      info.cops.visible  = nxbe_clipredraw;
      info.cops.obscured = nxbe_clipnull;
      info.wnd           = wnd;

#if CONFIG_NX_NPLANES > 1
      for (i = 0; i < be->vinfo.nplanes; i++)
        {
          nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                       &info.cops, &be->plane[i]);
        }
#else
      nxbe_clipper(wnd->above, &remaining, NX_CLIPORDER_DEFAULT,
                   &info.cops, &be->plane[0]);
#endif
    }
}
