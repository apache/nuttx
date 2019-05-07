/****************************************************************************
 * libs/libnx/nxmu/nx_openwindow.c
 *
 *   Copyright (C) 2008-2009, 2011-2013, 2019 Gregory Nutt. All rights
 *     reserved.
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

  wnd = (FAR struct nxbe_window_s *)lib_uzalloc(sizeof(struct nxbe_window_s));
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
