/****************************************************************************
 * graphics/nxbe/nxbe_isvisible.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_setvisibility_s
{
  struct nxbe_clipops_s cops;
  bool visible;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipvisible
 ****************************************************************************/

static void nxbe_clipvisible(FAR struct nxbe_clipops_s *cops,
                             FAR struct nxbe_plane_s *plane,
                             FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_setvisibility_s *info =
             (FAR struct nxbe_setvisibility_s *)cops;
  info->visible = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_isvisible
 *
 * Description:
 *   Return true if the point, pt, in window wnd is visible.  pt is in
 *   absolute screen coordinates
 *
 ****************************************************************************/

bool nxbe_isvisible(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_point_s *pos)
{
  struct nxbe_setvisibility_s info;

  /* Hidden windows are never visible */

  if (NXBE_ISHIDDEN(wnd))
    {
      return false;
    }

  /* Check if the absolute position lies within the window */

  if (!nxgl_rectinside(&wnd->bounds, pos))
    {
      return false;
    }

  /* If this is the top window, then the position is visible */

  if (!wnd->above)
    {
      return true;
    }

  /* The position within the window range, but the window is not at
   * the top.  We will have to work harder to determine if the point
   * visible
   */

  info.cops.visible  = nxbe_clipvisible;
  info.cops.obscured = nxbe_clipnull;
  info.visible       = false;

  nxbe_clipper(wnd->above, &wnd->bounds, NX_CLIPORDER_DEFAULT,
               &info.cops, &wnd->be->plane[0]);

  return info.visible;
}
