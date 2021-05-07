/****************************************************************************
 * libs/libnx/nxmu/nx_filltrapezoid.c
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

#include <string.h>
#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_filltrapezoid
 *
 * Description:
 *  Fill the specified trapezoidal region in the window with the specified
 *  color
 *
 * Input Parameters:
 *   hwnd  - The window handle
 *   clip - Clipping region (may be null)
 *   trap  - The trapezoidal region to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_filltrapezoid(NXWINDOW hwnd, FAR const struct nxgl_rect_s *clip,
                     FAR const struct nxgl_trapezoid_s *trap,
                     nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_filltrapezoid_s outmsg;
  int i;

#ifdef CONFIG_DEBUG_FEATURES
  /* Some debug-only sanity checks */

  if (wnd == NULL || trap == NULL || color == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Format the fill command */

  outmsg.msgid = NX_SVRMSG_FILLTRAP;
  outmsg.wnd   = wnd;

  /* If no clipping window was provided, then use the size of the entire
   * window
   */

  if (clip != NULL)
    {
      nxgl_rectcopy(&outmsg.clip, clip);
    }
  else
    {
      struct nxgl_rect_s tmpclip;

      /* Convert the window bounds to window relative coordinates */

      nxgl_rectoffset(&tmpclip, &wnd->bounds,
                     -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

      /* And use that for the clipping winding */

      nxgl_rectcopy(&outmsg.clip, &tmpclip);
    }

  /* Copy the trapezod and the color into the message */

  nxgl_trapcopy(&outmsg.trap, trap);

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < CONFIG_NX_NPLANES; i++)
#else
  i = 0;
#endif
    {
      outmsg.color[i] = color[i];
    }

  /* Forward the trapezoid fill command to the server */

  return nxmu_sendwindow(wnd, &outmsg,
                         sizeof(struct nxsvrmsg_filltrapezoid_s));
}
