/****************************************************************************
 * libs/libnx/nxmu/nx_requestbkgd.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_requestbkgd
 *
 * Description:
 *   NX normally controls a separate window called the background window.
 *   It repaints the window as necessary using only a solid color fill.  The
 *   background window always represents the entire screen and is always
 *   below other windows.  It is useful for an application to control the
 *   background window in the following conditions:
 *
 *   - If you want to implement a windowless solution.  The single screen
 *     can be used to creat a truly simple graphic environment.
 *   - When you want more on the background than a solid color.  For
 *     example, if you want an image in the background, or animations in the
 *     background, or live video, etc.
 *
 *   This API only requests the handle of the background window.  That
 *   handle will be returned asynchronously in a subsequent position and
 *   redraw callbacks.
 *
 *
 *   Cautions:
 *   - The following should never be called using the background window.
 *     They are guaranteed to cause severe crashes:
 *
 *       nx_setposition, nx_setsize, nx_raise, nx_lower.
 *
 *   - Neither nx_opengbwindow or nx_closebgwindow should be called more than
 *     once.  Multiple instances of the background window are not supported.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   cb     - Callbacks to use for processing background window events
 *   arg    - User provided argument (see nx_openwindow, nx_constructwindow)
 *
 * Returned Value:
 *   OK: Success; ERROR of failure with errno set appropriately.
 *
 ****************************************************************************/

int nx_requestbkgd(NXHANDLE handle, FAR const struct nx_callback_s *cb,
                   FAR void *arg)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_requestbkgd_s outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn || !cb)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Request access to the background window from the server */

  outmsg.msgid = NX_SVRMSG_REQUESTBKGD;
  outmsg.conn  = conn;
  outmsg.cb    = cb;
  outmsg.arg   = arg;

  return nxmu_sendserver(conn, &outmsg,
                         sizeof(struct nxsvrmsg_requestbkgd_s));
}
