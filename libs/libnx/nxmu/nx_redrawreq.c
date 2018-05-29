/****************************************************************************
 * libs/libnx/nxmu/nx_redrawreq.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_redrawreq
 *
 * Description:
 *   This will cause a NX re-draw callback to the client that owns the
 *   window.  This is not normally called from user code, but may be
 *   used within middle-ware layers when redrawing is needed.
 *
 * Input Parameters:
 *   hwnd - Window handle
 *   rect - The rectangle that needs to be re-drawn (in window relative
 *          coordinates)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nx_redrawreq(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_redrawreq_s outmsg;
  int ret;

  DEBUGASSERT(wnd && rect);

  /* Inform the server of the changed position */

  outmsg.msgid = NX_SVRMSG_REDRAWREQ;
  outmsg.wnd   = wnd;
  nxgl_rectcopy(&outmsg.rect, rect);

  ret = nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_redrawreq_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendwindow failed: %d\n", errno);
    }
}
