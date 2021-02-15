/****************************************************************************
 * libs/libnx/nxmu/nx_redrawreq.c
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
      gerr("ERROR: nxmu_sendwindow failed: %d\n", get_errno());
    }
}
