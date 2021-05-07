/****************************************************************************
 * graphics/nxmu/nxmu_reportposition.c
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

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_reportposition
 *
 * Description:
 *   Report the new size/position of the window.
 *
 ****************************************************************************/

void nxmu_reportposition(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxclimsg_newposition_s outmsg;
  int ret;

  /* Send the size/position info */

  outmsg.msgid  = NX_CLIMSG_NEWPOSITION;
  outmsg.wnd    = wnd;
  outmsg.pos.x  = wnd->bounds.pt1.x;
  outmsg.pos.y  = wnd->bounds.pt1.y;

  nxgl_rectsize(&outmsg.size, &wnd->bounds);

  /* Provide the background window bounding box which is the screen limits
   * It must always have (0,0) as its origin
   */

  nxgl_rectcopy(&outmsg.bounds, &wnd->be->bkgd.bounds);

  /* And provide this to the client */

  ret = nxmu_sendclientwindow(wnd,
                              &outmsg,
                              sizeof(struct nxclimsg_newposition_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendclient failed: %d\n", get_errno());
    }
}
