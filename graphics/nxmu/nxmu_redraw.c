/****************************************************************************
 * graphics/nxmu/nxmu_redraw.c
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

#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_redraw_pwfb
 *
 * Description:
 *   Redraw into the per-window framebuffer
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static inline void nxmu_redraw_pwfb(FAR struct nxbe_window_s *wnd,
                                    FAR const struct nxgl_rect_s *rect)
{
  FAR const void *src[CONFIG_NX_NPLANES];
  struct nxgl_rect_s wndrect;
  struct nxgl_point_s origin;
  unsigned int bpp;

  /* Put the rectangle back relative to the window */

  nxgl_rectoffset(&wndrect, rect,
                  -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  /* Get the source of address of the rectangle in the framebuffer. */

  bpp    = wnd->be->plane[0].pinfo.bpp;
  src[0] = (FAR const void *)((FAR uint8_t *)wnd->fbmem +
                               wndrect.pt1.y * wnd->stride +
                               ((bpp * wndrect.pt1.x) >> 3));

  /* For resolutions less than 8-bits, the starting pixel will be contained
   * in the byte pointed to by src[0]but may not be properly aligned for the
   * transfer.  We fix this by modifying the origin.
   */

  origin.x = wndrect.pt1.x;
  origin.y = wndrect.pt1.y;

  switch (bpp)
    {
#ifndef CONFIG_NX_DISABLE_1BPP
      case 1:  /* 1 bit per pixel */
        {
          origin.x &= ~7;
        }
        break;
#endif

#ifndef CONFIG_NX_DISABLE_2BPP
      case 2:  /* 2 bits per pixel */
        {
          origin.x &= ~3;
        }
        break;
#endif

#ifndef CONFIG_NX_DISABLE_4BPP
      case 4:  /* 4 bits per pixel */
        {
          origin.x &= ~1;
        }
        break;
#endif

      default:
        break;
    }

  /* And render the bitmap into device graphics memory */

  nxbe_flush(wnd, &wndrect, src, &origin, wnd->stride);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_redrawreq
 *
 * Description:
 *  Send a message to the client requesting that it to redraw the rectangular
 *  region in the window.
 *
 ****************************************************************************/

void nxmu_redrawreq(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_rect_s *rect)
{
  struct nxclimsg_redraw_s outmsg;

  /* Send the client redraw message */

  outmsg.msgid = NX_CLIMSG_REDRAW;
  outmsg.wnd   = wnd;
  outmsg.more  = false;
  nxgl_rectoffset(&outmsg.rect, rect,
                  -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  nxmu_sendclientwindow(wnd, &outmsg,
                        sizeof(struct nxclimsg_redraw_s));
}

/****************************************************************************
 * Name: nxmu_redraw
 *
 * Description:
 *   Redraw client window data.  This may involve either sending a message
 *   to the client requesting that it redraw a region of the window.  Or, in
 *   the base that the window supports a per-window framebuffer, this might
 *   amount to an immediate redraw from the framebuffer.
 *
 ****************************************************************************/

void nxmu_redraw(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect)
{
  /* Don't update hidden windows */

  if (!NXBE_ISHIDDEN(wnd))
    {
#ifdef CONFIG_NX_RAMBACKED
      /* If this window supports a pre-window frame buffer, then we can just
       * update the device content from that framebuffer.
       */

      if (NXBE_ISRAMBACKED(wnd))
        {
          nxmu_redraw_pwfb(wnd, rect);
        }

      /* Otherwise, send a message to the client requesting an update of the
       * affected region in the window.
       */

      else
#endif
        {
          nxmu_redrawreq(wnd, rect);
        }
    }
}
