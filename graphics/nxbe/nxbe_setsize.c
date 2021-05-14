/****************************************************************************
 * graphics/nxbe/nxbe_setsize.c
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
#include <debug.h>

#ifdef CONFIG_NX_RAMBACKED
#  include <string.h>
#ifdef CONFIG_BUILD_KERNEL
#  include <nuttx/pgalloc.h>
#else
#  include <nuttx/kmalloc.h>
#endif
#endif

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) ((a > b) ? a : b)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_realloc
 *
 * Description:
 *   After the display size has changed, reallocate the pre-window frame
 *   buffer for the new framebuffer size.
 *
 *   REVISIT:  This currently takes a brute force force approach, allocating
 *   the new framebuffer while the old framebuffer is still in place.  There
 *   may be some clever way to do this reallocation in place.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static void nxbe_realloc(FAR struct nxbe_window_s *wnd,
                         FAR struct nxgl_rect_s *oldbounds)
{
  FAR nxgl_mxpixel_t *newfb;
  FAR uint8_t *src;
  FAR uint8_t *dest;
  struct nxgl_rect_s bounds;
  nxgl_coord_t minheight;
  nxgl_coord_t newwidth;
  nxgl_coord_t newheight;
  nxgl_coord_t oldwidth;
  nxgl_coord_t oldheight;
  nxgl_coord_t row;
  size_t newfbsize;
  unsigned int minstride;
  unsigned int newstride;
  unsigned int bpp;
#ifdef CONFIG_BUILD_KERNEL
  unsigned int newnpages;
#endif

  /* Allocate framebuffer memory if the per-window framebuffer feature has
   * been selected.
   *
   * REVISIT:  This initial state of the framebuffer is uninitialized and
   * not synchronized with the graphic device content.  It will take a full
   * screen update from the application to force the framebuffer and device
   * to be consistent.
   */

  if (NXBE_ISRAMBACKED(wnd))
    {
      oldwidth        = oldbounds->pt2.x - oldbounds->pt1.x + 1;
      oldheight       = oldbounds->pt2.y - oldbounds->pt1.y + 1;

      newwidth        = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
      newheight       = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

      bpp             = wnd->be->plane[0].pinfo.bpp;
      newstride       = (bpp * newwidth + 7) >> 3;
      newfbsize       = newstride * newheight;

#ifdef CONFIG_BUILD_KERNEL
      /* Re-allocate memory from the page pool. */

      /* Determine the number of pages to be allocated. */

      newnpages = (uint16_t)MM_NPAGES(newfbsize);

      /* Allocate the pages */

      newfb = (FAR nxgl_mxpixel_t *)mm_pgalloc(newnpages);
      if (newfb == NULL)
        {
          /* Fall back to no RAM back up */

          gerr("ERROR: mm_pgalloc() failed for fbsize=%lu, npages=%u\n",
               (unsigned long)newfbsize, npages);

          mm_pgfree(wnd->fbmem, wnd->npages);
          wnd->stride = 0;
          wnd->npages = 0;
          wnd->fbmem  = NULL;
          NXBE_CLRRAMBACKED(wnd);
          return;
        }
#else
      /* Re-allocate memory from the user space heap. */

      newfb = (FAR nxgl_mxpixel_t *)kumm_malloc(newfbsize);
      if (newfb == NULL)
        {
          /* Fall back to no RAM back up */

          gerr("ERROR: kumm_malloc() failed for fbsize=%lu\n",
               (unsigned long)newfbsize);

          kumm_free(wnd->fbmem);
          wnd->stride = 0;
          wnd->fbmem  = NULL;
          NXBE_CLRRAMBACKED(wnd);
          return;
        }
#endif

      /* Copy the content of the old framebuffer to the new frame buffer */

      minheight = MIN(oldheight, newheight);
      minstride = MIN(wnd->stride, newstride);

      /* Process each line one at a time */

      for (src  = (FAR uint8_t *)wnd->fbmem, dest = (FAR uint8_t *)newfb,
           row = 0;
           row < minheight;
           src += wnd->stride, dest += newstride, row++)
        {
          /* Copy valid row data */

          memcpy(dest, src, minstride);

          /* Pad any extra pixel data on the right (with zeroes?) */

          if (minstride < newstride)
            {
              memset(dest + minstride, 0, newstride - minstride);
            }
        }

      /* Pad any extra lines at the bottom (with zeroes?) */

      if (row < newheight)
        {
          size_t nbytes = newstride * (newheight - row);
          memset(dest, 0, nbytes);
        }

      /* Free the old framebuffer and configure for the new framebuffer */

#ifdef CONFIG_BUILD_KERNEL
      mm_pgfree(wnd->fbmem, wnd->npages);
      wnd->npages = newnpages;
#else
      kumm_free(wnd->fbmem);
#endif
      wnd->stride = newstride;
      wnd->fbmem  = newfb;

      /* If the window became wider, then send a message requesting an update
       * of the new territory on the right.
       */

      if (oldwidth < newwidth)
        {
          /* Get a bounding box in device coordinates */

          bounds.pt1.x = wnd->bounds.pt1.x + oldwidth;
          bounds.pt1.y = wnd->bounds.pt1.y;
          bounds.pt2.x = wnd->bounds.pt2.x;
          bounds.pt2.y = wnd->bounds.pt2.y + MIN(oldheight, newheight) - 1;

          /* Send the redraw request */

          nxmu_redrawreq(wnd, &bounds);
        }

      /* If the window became taller, then send a message requesting an
       * update of the new territory at the bottom.
       */

      if (oldheight < newheight)
        {
          /* Get a bounding box in device coordinates */

          bounds.pt1.x = wnd->bounds.pt1.x;
          bounds.pt1.y = wnd->bounds.pt1.y + oldheight;
          bounds.pt2.x = wnd->bounds.pt2.x;
          bounds.pt2.y = wnd->bounds.pt2.y;

          /* Send the redraw request */

          nxmu_redrawreq(wnd, &bounds);
        }
    }
}
#else
#  define nxbe_realloc(w,b)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setsize
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the size of a window.
 *
 ****************************************************************************/

void nxbe_setsize(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_size_s *size)
{
  struct nxgl_rect_s bounds;

  DEBUGASSERT(wnd != NULL && size != NULL);

  /* Save the 'before' size of the window's bounding box */

  nxgl_rectcopy(&bounds, &wnd->bounds);

  /* Add the window origin to the supplied size get the new window bounding
   * box
   */

  wnd->bounds.pt2.x = wnd->bounds.pt1.x + size->w - 1;
  wnd->bounds.pt2.y = wnd->bounds.pt1.y + size->h - 1;

  /* Clip the new bounding box so that lies within the background screen */

  nxgl_rectintersect(&wnd->bounds, &wnd->bounds, &wnd->be->bkgd.bounds);

  /* Report the new size/position.  The application needs to know the new
   * size before getting redraw requests.
   */

  nxmu_reportposition(wnd);

  /* Re-allocate the per-window framebuffer memory for the new window size. */

  nxbe_realloc(wnd, &bounds);

  /* We need to update the larger of the two regions described by the
   * original bounding box and the new bounding box.   That will be the
   * union of the two bounding boxes.
   */

  nxgl_rectunion(&bounds, &bounds, &wnd->bounds);

  /* Then redraw this window AND all windows below it. Having resized the
   * window, we may have exposed previoulsy obscured portions of windows
   * below this one.
   */

  nxbe_redrawbelow(wnd->be, wnd, &bounds);
}
