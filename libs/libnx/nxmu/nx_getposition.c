/****************************************************************************
 * libs/libnx/nxmu/nx_getposition.c
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
 * Name: nx_getposition
 *
 * Description:
 *  Request the position and size information for the selected window.  The
 *  values will be return asynchronously through the client callback function
 *  pointer.
 *
 * Input Parameters:
 *   hwnd   - The window handle
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_getposition(NXWINDOW hwnd)
{
  FAR struct nxbe_window_s     *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_getposition_s outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (!wnd)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Request the size/position info.
   *
   * It is tempting to just take the positional information from the window
   * structure that we have in our hands now.  However, we need to run this
   * through the server to keep things serialized.  There might, for example,
   * be a pending size/position change and, in that case, this function would
   * return the wrong info.
   */

  outmsg.msgid = NX_SVRMSG_GETPOSITION;
  outmsg.wnd   = wnd;

  return nxmu_sendwindow(wnd, &outmsg,
                         sizeof(struct nxsvrmsg_getposition_s));
}
