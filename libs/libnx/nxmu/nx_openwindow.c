/****************************************************************************
 * libs/libnx/nxmu/nx_openwindow.c
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

#include "nxcontext.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_openwindow
 *
 * Description:
 *   Create a new window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   flags  - Optional flags.  These include:
 *            NXBE_WINDOW_RAMBACKED:  Creates a RAM backed window.  This
 *              option is only valid if CONFIG_NX_RAMBACKED is enabled.
 *            NXBE_WINDOW_HIDDEN:  The window is create in the HIDDEN state
 *             and can be made visible later with nx_setvisibility().
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NX callbacks.
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXWINDOW nx_openwindow(NXHANDLE handle, uint8_t flags,
                       FAR const struct nx_callback_s *cb, FAR void *arg)
{
  FAR struct nxbe_window_s *wnd;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (handle == NULL || cb == NULL || (flags & ~NX_WINDOW_USER) != 0)
    {
      set_errno(EINVAL);
      return NULL;
    }
#endif

  /* Pre-allocate the window structure */

  wnd = (FAR struct nxbe_window_s *)
        lib_uzalloc(sizeof(struct nxbe_window_s));
  if (!wnd)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  /* Then let nx_constructwindow do the rest */

  ret = nx_constructwindow(handle, (NXWINDOW)wnd, flags, cb, arg);
  if (ret < 0)
    {
      /* An error occurred, the window has been freed */

      return NULL;
    }

  /* Return the uninitialized window reference.  Since the server
   * serializes all operations, we can be assured that the window will
   * be initialized before the first operation on the window.
   */

  return (NXWINDOW)wnd;
}
