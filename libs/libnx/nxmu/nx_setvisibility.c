/****************************************************************************
 * libs/libnx/nxmu/nx_setvisibility.c
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
 * Name: nx_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present and will update normally, but will not be visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   hwnd - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_setvisibility(NXWINDOW hwnd, bool hide)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_setvisibility_s outmsg;

  /* Send the MODAL message */

  outmsg.msgid = NX_SVRMSG_SETVISIBILITY;
  outmsg.wnd   = wnd;
  outmsg.hide  = hide;

  return nxmu_sendwindow(wnd, &outmsg,
                         sizeof(struct nxsvrmsg_setvisibility_s));
}
