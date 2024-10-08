/****************************************************************************
 * libs/libnx/nxmu/nx_closewindow.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_closewindow
 *
 * Description:
 *   Destroy a window created by nx_openwindow.
 *
 * Input Parameters:
 *   wnd - The window to be destroyed
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_closewindow(NXWINDOW hwnd)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_closewindow_s outmsg;

  /* Request destruction of the window by the server */

  outmsg.msgid = NX_SVRMSG_CLOSEWINDOW;
  outmsg.wnd   = wnd;

  return nxmu_sendserver(wnd->conn, &outmsg,
                         sizeof(struct nxsvrmsg_closewindow_s));
}
