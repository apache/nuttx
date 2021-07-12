/****************************************************************************
 * graphics/nxmu/nxmu_openwindow.c
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
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_NX_RAMBACKED
#ifdef CONFIG_BUILD_KERNEL
#  include "nuttx/pgalloc.h"
#else
#  include "nuttx/kmalloc.h"
#endif
#  include <nuttx/nx/nxbe.h>
#endif

#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   be  - The back-end status structure
 *   wnd  - The pre-allocated window structure to be initialized [IN/OUT]
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_openwindow(FAR struct nxbe_state_s *be,
                     FAR struct nxbe_window_s *wnd)
{
#ifdef CONFIG_NX_RAMBACKED
  nxgl_coord_t width;
  nxgl_coord_t height;
  size_t fbsize;
  unsigned int bpp;
#endif

  /* The window structure was allocated in nx_openwindow and all fields have
   * been set to zero; conn, flags, cb, and arg which were initialized on
   * the client side.  On the server side, we need only initialize a few
   * more the non zero fields and insert the new window at the top of the
   * display.
   */

  wnd->be           = be;

#ifdef CONFIG_NX_RAMBACKED
  /* Allocate framebuffer memory if the per-window framebuffer feature has
   * been selected.
   *
   * REVISIT:  This initial state of the framebuffer is uninitialized and
   * not synchronized with the graphic device content.  It will take a full
   * screen update from the application to force the framebuffer and device
   * to be consistent.
   *
   * REVISIT:  Assumes a single color plane.
   */

  if (NXBE_ISRAMBACKED(wnd))
    {
      width         = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
      height        = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;
      bpp           = wnd->be->plane[0].pinfo.bpp;
      wnd->stride   = (bpp * width + 7) >> 3;
      fbsize        = wnd->stride * height;

#ifdef CONFIG_BUILD_KERNEL
      /* Allocate memory from the page pool because:
       *
       *   1) The page pool is the largest memory pool and best for
       *      framebuffers.
       *   2) The allocation will be contiguous and much easier to
       *      map into a user address later.
       */

      /* Determine the number of pages to be allocated */

      wnd->npages = (uint16_t)MM_NPAGES(fbsize);

      /* Allocate the pages */

      wnd->fbmem = (FAR nxgl_mxpixel_t *)mm_pgalloc(wnd->npages);
      if (wnd->fbmem == NULL)
        {
          /* Fall back to no RAM back up */

          gerr("ERROR: mm_pgalloc() failed for fbsize=%lu, npages=%u\n",
               (unsigned long)fbsize, wnd->npages);

          wnd->stride = 0;
          wnd->npages = 0;
          NXBE_CLRRAMBACKED(wnd);
        }
#else
      /* Allocate memory from the user space heap because:
       *
       *   1) The  user space heap is the larger memory pool and best for
       *      framebuffers (protected mode).
       *   2) The user space heap is openly accessible at all privilege
       *      levels.
       */

      wnd->fbmem = (FAR nxgl_mxpixel_t *)kumm_malloc(fbsize);
      if (wnd->fbmem == NULL)
        {
          /* Fall back to no RAM back up */

          gerr("ERROR: kumm_malloc() failed for fbsize=%lu\n",
               (unsigned long)fbsize);

          wnd->stride = 0;
          NXBE_CLRRAMBACKED(wnd);
        }
#endif
    }
#endif

  /* Is the window being created in the hidden state? */

  if (!NXBE_ISHIDDEN(wnd))
    {
      /* No.. Insert the new window at the correct position in the
       * hierarchy.  topwnd is never NULL (it may point only at the
       * background window, however).  If we are in a modal state, then we
       * cannot insert the window at the top of the display.
       */

      if (NXBE_STATE_ISMODAL(be) && be->topwnd->below != NULL)
        {
          /* We are in a modal state.  The topwnd is not the background and
           * it has focus.
           */

          wnd->above        = be->topwnd;
          wnd->below        = be->topwnd->below;

          be->topwnd->below = wnd;
        }
      else
        {
          /* Otherwise insert the new window at the top on the display. */

          wnd->above        = NULL;
          wnd->below        = be->topwnd;

          be->topwnd->above = wnd;
          be->topwnd        = wnd;
        }
    }

  /* Report the initial size/position of the window to the client */

  nxmu_reportposition(wnd);

#ifdef CONFIG_NX_XYINPUT
  /* Provide the initial mouse settings to the client */

  nxmu_mousereport(wnd);
#endif
}
