/****************************************************************************
 * nuttx/graphics/nxterm/nxterm_resize.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 * Name: nxterm_resize
 *
 * Description:
 *   This function handles the IOCTL resize command.  It indicates that the
 *   size of the NxTerm window has changed and needs to be updated.
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   size   - The new window size
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int nxterm_resize(NXTERM handle, FAR const struct nxgl_size_s *size)
{
  FAR struct nxterm_state_s *priv;
  int ret;

  DEBUGASSERT(handle != NULL && size != NULL);
  ginfo("size={%d,%d)\n", size->w, size->h);

  /* Recover our private state structure */

  priv = (FAR struct nxterm_state_s *)handle;

  /* Get exclusive access to the state structure */

  ret = nxterm_semwait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the new window size.
   * REVISIT:  Should other things be reset as well?
   */

  priv->wndo.wsize.w = size->w;
  priv->wndo.wsize.h = size->h;

  nxterm_sempost(priv);
  return true;
}
