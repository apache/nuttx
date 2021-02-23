/****************************************************************************
 * libs/libnx/nxmu/nx_move.c
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
 * Name: nx_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   hwnd   - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_move(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
            FAR const struct nxgl_point_s *offset)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_move_s    outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (!wnd)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Format the fill command */

  outmsg.msgid      = NX_SVRMSG_MOVE;
  outmsg.wnd        = wnd;
  outmsg.offset.x   = offset->x;
  outmsg.offset.y   = offset->y;

  nxgl_rectcopy(&outmsg.rect, rect);

  /* Forward the fill command to the server */

  return nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_move_s));
}
