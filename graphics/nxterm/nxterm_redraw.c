/****************************************************************************
 * graphics/nxterm/nxterm_redraw.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#include "nxterm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_redraw
 *
 * Description:
 *   Re-draw a portion of the NX console.  This function should be called
 *   from the appropriate window callback logic.
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   rect - The rectangle that needs to be re-drawn (in window relative
 *          coordinates)
 *   more - true:  More re-draw requests will follow
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxterm_redraw(NXTERM handle,
                   FAR const struct nxgl_rect_s *rect,
                   bool more)
{
  FAR struct nxterm_state_s *priv;
  int ret;
  int i;

  DEBUGASSERT(handle && rect);
  ginfo("rect={(%d,%d),(%d,%d)} more=%s\n",
        rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
        more ? "true" : "false");

  /* Recover our private state structure */

  priv = (FAR struct nxterm_state_s *)handle;

  /* Get exclusive access to the state structure */

  do
    {
      ret = nxmutex_lock(&priv->lock);
    }
  while (ret < 0);

  /* Fill the rectangular region with the window background color */

  ret = priv->ops->fill(priv, rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gerr("ERROR: fill failed: %d\n", get_errno());
    }

  /* Then redraw each character on the display (Only the characters within
   * the rectangle will actually be redrawn).
   */

  for (i = 0; i < priv->nchars; i++)
    {
      nxterm_fillchar(priv, rect, &priv->bm[i]);
    }

  nxmutex_unlock(&priv->lock);
}
