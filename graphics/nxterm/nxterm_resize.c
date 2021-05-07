/****************************************************************************
 * graphics/nxterm/nxterm_resize.c
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
