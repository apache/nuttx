/****************************************************************************
 * graphics/nxmu/nxmu_openwindow.c
 *
 *   Copyright (C) 2008-2011, 2019 Gregory Nutt. All rights reserved.
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

void nxmu_openwindow(FAR struct nxbe_state_s *be, FAR struct nxbe_window_s *wnd)
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
