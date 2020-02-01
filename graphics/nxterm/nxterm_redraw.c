/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_redraw.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

void nxterm_redraw(NXTERM handle, FAR const struct nxgl_rect_s *rect, bool more)
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
      ret = nxterm_semwait(priv);
    }
  while (ret < 0);

  /* Fill the rectangular region with the window background color */

  ret = priv->ops->fill(priv, rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gerr("ERROR: fill failed: %d\n", errno);
    }

  /* Then redraw each character on the display (Only the characters within
   * the rectangle will actually be redrawn).
   */

  for (i = 0; i < priv->nchars; i++)
    {
      nxterm_fillchar(priv, rect, &priv->bm[i]);
    }

  nxterm_sempost(priv);
}
