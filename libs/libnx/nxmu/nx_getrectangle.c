/****************************************************************************
 * libs/libnx/nxmu/nx_getrectangle.c
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

#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_getrectangle(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                    unsigned int plane, FAR uint8_t *dest,
                    unsigned int deststride)
{
  FAR struct nxbe_window_s        *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_getrectangle_s  outmsg;
  int ret;
  sem_t sem_done;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hwnd || !rect || !dest)
    {
      ginfo("Invalid parameters\n");
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Format the fill command */

  outmsg.msgid      = NX_SVRMSG_GETRECTANGLE;
  outmsg.wnd        = wnd;
  outmsg.plane      = plane;
  outmsg.dest       = dest;
  outmsg.deststride = deststride;

  nxgl_rectcopy(&outmsg.rect, rect);

  /* Create a semaphore for tracking command completion */

  outmsg.sem_done = &sem_done;

  ret = _SEM_INIT(&sem_done, 0, 0);
  if (ret < 0)
    {
      gerr("ERROR: _SEM_INIT failed: %d\n", _SEM_ERRNO(ret));
      return ret;
    }

  /* Forward the fill command to the server */

  ret = nxmu_sendwindow(wnd, &outmsg,
                        sizeof(struct nxsvrmsg_getrectangle_s));

  /* Wait that the command is completed, so that caller can release the
   * buffer.
   */

  if (ret == OK)
    {
      ret = _SEM_WAIT(&sem_done);
    }

  /* Destroy the semaphore and return. */

  _SEM_DESTROY(&sem_done);

  return ret;
}
