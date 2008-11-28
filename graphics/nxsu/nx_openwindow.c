/****************************************************************************
 * graphics/nxsu/nx_openwindow.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx.h>
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   cb     - Callbacks used to process windo events
 *
 * Return:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXWINDOW nx_openwindow(NXHANDLE handle, FAR const struct nx_callback_s *cb)
{
  FAR struct nxfe_state_s *fe = (FAR struct nxfe_state_s *)handle;
  FAR struct nxbe_state_s *be = &fe->be;
  FAR struct nxbe_window_s *wnd;

#ifdef CONFIG_DEBUG
  if (!fe || !cb)
    {
      errno = EINVAL;
      return NULL;
    }
#endif

  /* Pre-allocate the window structure */

  wnd = (FAR struct nxbe_window_s *)zalloc(sizeof(struct nxbe_window_s));
  if (!wnd)
    {
      errno = ENOMEM;
      return NULL;
    }

  /* Initialize the window structure */

  wnd->be           = be;
  wnd->cb           = cb;

  /* Insert the new window at the top on the display.  topwind is
   * never NULL (it may point only at the background window, however)
   */

  wnd->above        = NULL;
  wnd->below        = be->topwnd;

  be->topwnd->above = wnd;
  be->topwnd        = wnd;

  /* Report the initialize size/position of the window */

  nxfe_reportposition((NXWINDOW)wnd);

  /* Provide the initial mouse settings */

#ifdef CONFIG_NX_MOUSE
  nxsu_mousereport(wnd);
#endif

  /* Return the initialized window reference */

  return (NXWINDOW)wnd;
}

