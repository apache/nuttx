/****************************************************************************
 * libs/libnx/nxtk/nxtk_openwindow.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>

#include "nxcontext.h"
#include "nxtk.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

nxgl_mxpixel_t g_bordercolor1[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR1
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

nxgl_mxpixel_t g_bordercolor2[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR2
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

nxgl_mxpixel_t g_bordercolor3[CONFIG_NX_NPLANES] =
{
  CONFIG_NXTK_BORDERCOLOR3
#if CONFIG_NX_NPLANES > 1
#  error "Multiple plane border colors not defined"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_openwindow
 *
 * Description:
 *   Create a new, framed window.
 *
 * Input Parameters:
 *   handle - The handle returned by nx_connect
 *   flags  - Optional flags.  These include:
 *            NXBE_WINDOW_RAMBACKED:  Creates a RAM backed window.  This
 *              option is only valid if CONFIG_NX_RAMBACKED is enabled.
 *            NXBE_WINDOW_HIDDEN:  The window is create in the HIDDEN state
 *             and can be made visible later with nxtk_setvisibility().
 *   cb     - Callbacks used to process window events
 *   arg    - User provided value that will be returned with NXTK callbacks.
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NXTK window accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXTKWINDOW nxtk_openwindow(NXHANDLE handle, uint8_t flags,
                           FAR const struct nx_callback_s *cb,
                           FAR void *arg)
{
  FAR struct nxtk_framedwindow_s *fwnd;
  int ret;

#ifdef CONFIG_DEBUG_FEATURES
  if (handle == NULL || cb == NULL ||  (flags & ~NXTK_WINDOW_USER) != 0)
    {
      set_errno(EINVAL);
      return NULL;
    }
#endif

  /* Pre-allocate the window structure */

  fwnd = (FAR struct nxtk_framedwindow_s *)
    lib_uzalloc(sizeof(struct nxtk_framedwindow_s));

  if (fwnd == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  /* Initialize the window structure */

  fwnd->fwcb  = cb;
  fwnd->fwarg = arg;

  /* Then let nx_constructwindow do the rest */

  ret = nx_constructwindow(handle, (NXWINDOW)&fwnd->wnd,
                           flags | NXBE_WINDOW_FRAMED, &g_nxtkcb, NULL);
  if (ret < 0)
    {
      /* An error occurred, the window has been freed */

      return NULL;
    }

  /* Return the initialized window reference */

  return (NXTKWINDOW)fwnd;
}
