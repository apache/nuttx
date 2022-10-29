/****************************************************************************
 * libs/libnx/nxmu/nx_bitmap.c
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

#include <nuttx/semaphore.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   hwnd   - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in pixels.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_bitmap(NXWINDOW hwnd, FAR const struct nxgl_rect_s *dest,
              FAR const void *src[CONFIG_NX_NPLANES],
              FAR const struct nxgl_point_s *origin, unsigned int stride)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_bitmap_s outmsg;
  int i;
  int ret;
  sem_t sem_done;

#ifdef CONFIG_DEBUG_FEATURES
  if (!wnd || !dest || !src || !origin)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Format the bitmap command */

  outmsg.msgid      = NX_SVRMSG_BITMAP;
  outmsg.wnd        = wnd;
  outmsg.stride     = stride;

  for (i = 0; i < CONFIG_NX_NPLANES; i++)
    {
      outmsg.src[i] = src[i];
    }

  outmsg.origin.x   = origin->x;
  outmsg.origin.y   = origin->y;
  nxgl_rectcopy(&outmsg.dest, dest);

  /* Create a semaphore for tracking command completion */

  outmsg.sem_done = &sem_done;

  ret = _SEM_INIT(&sem_done, 0, 0);
  if (ret < 0)
    {
      gerr("ERROR: _SEM_INIT failed: %d\n", _SEM_ERRNO(ret));
      return ret;
    }

  /* Forward the fill command to the server */

  ret = nxmu_sendwindow(wnd, &outmsg, sizeof(struct nxsvrmsg_bitmap_s));

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
