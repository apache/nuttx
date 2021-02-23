/****************************************************************************
 * libs/libnx/nxmu/nx_setsize.c
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
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_setsize
 *
 * Description:
 *  Set the size of the selected window
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *   size   - The new size of the window.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setsize(NXWINDOW hwnd, FAR const struct nxgl_size_s *size)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_setsize_s outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (wnd == NULL || size == NULL || size->w < 0 || size->h < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Then inform the server of the changed position */

  outmsg.msgid  = NX_SVRMSG_SETSIZE;
  outmsg.wnd    = wnd;
  outmsg.size.w = size->w;
  outmsg.size.h = size->h;

  return nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_setsize_s));
}
