/****************************************************************************
 * graphics/nxmu/nxmu_redrawreq.c
 *
 *   Copyright (C) 2008-2009, 2011-2012, 2019 Gregory Nutt. All rights
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_redrawreq
 *
 * Description:
 *   Request the client that has this window to redraw the rectangular region.
 *
 ****************************************************************************/

void nxmu_redrawreq(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_rect_s *rect)
{
#ifdef CONFIG_NX_RAMBACKED
  /* If this window supports a pre-window frame buffer, then we can just
   * update the device content from that framebuffer.
   */

  if (NXBE_ISRAMBACKED(wnd))
    {
      FAR const void *src[CONFIG_NX_NPLANES] =
      {
        (FAR const void *)wnd->fbmem
      };
      struct nxgl_point_s origin =
      {
        0, 0
      };

      nxbe_bitmap_dev(wnd, rect, src, &origin, wnd->stride);
    }
  else
#endif
    {
      struct nxclimsg_redraw_s outmsg;

      /* Send the client redraw message */

      outmsg.msgid = NX_CLIMSG_REDRAW;
      outmsg.wnd   = wnd;
      outmsg.more  = false;
      nxgl_rectoffset(&outmsg.rect, rect,
                      -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

      (void)nxmu_sendclientwindow(wnd, &outmsg,
                                  sizeof(struct nxclimsg_redraw_s));
    }
}

