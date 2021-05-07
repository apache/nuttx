/****************************************************************************
 * libs/libnx/nxmu/nx_constructwindow.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

#include "nxcontext.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_constructwindow
 *
 * Description:
 *   This function is the same a nx_openwindow EXCEPT that the client
 *   provides the window structure instance.  nx_constructwindow will
 *   initialize the the pre-allocated window structure for use by NX.
 *   This function is provided in addition to nx_openwindow in order
 *   to support a kind of inheritance:  The caller's window structure
 *   may include extensions that are not visible to NX.
 *
 *   NOTE:
 *   hwnd must have been allocated using a user-space allocator that permits
 *   user access to the window.  Once provided to nx_constructwindow() that
 *   memory is owned and managed by NX.  On certain error conditions or
 *   when the window is closed, NX will free the window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   hwnd   - The pre-allocated window structure.
 *   flags  - Optional flags.  Must be zero unless CONFIG_NX_RAMBACKED is
 *            enabled.  In that case, it may be zero or
 *            NXBE_WINDOW_RAMBACKED
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately.  In the
 *   case of ERROR, NX will have deallocated the pre-allocated window.
 *
 ****************************************************************************/

int nx_constructwindow(NXHANDLE handle, NXWINDOW hwnd, uint8_t flags,
                       FAR const struct nx_callback_s *cb, FAR void *arg)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_openwindow_s outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (wnd == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (conn == NULL || cb == NULL || (flags & ~NXBE_WINDOW_USER) != 0)
    {
      lib_ufree(wnd);
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Setup only the connection structure, user flags, callbacks and client
   * private data reference. The server will set everything else up.
   */

  wnd->conn   = conn;
  wnd->flags  = flags;
  wnd->cb     = cb;
  wnd->arg    = arg;
#ifdef CONFIG_NX_RAMBACKED
#ifdef CONFIG_BUILD_KERNEL
  wnd->npages = 0;
#endif
  wnd->stride = 0;
  wnd->fbmem  = NULL;
#endif

  /* Request initialization the new window from the server */

  outmsg.msgid = NX_SVRMSG_OPENWINDOW;
  outmsg.wnd   = wnd;

  return nxmu_sendserver(conn, &outmsg,
                         sizeof(struct nxsvrmsg_openwindow_s));
}
