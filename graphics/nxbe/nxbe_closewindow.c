/****************************************************************************
 * graphics/nxbe/nxbe_closewindow.c
 *
 *   Copyright (C) 2008-2009, 2011, 2019 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxglib.h>

#if defined(CONFIG_NX_RAMBACKED) && defined(CONFIG_BUILD_KERNEL)
#  include "nuttx/pgalloc.h"
#endif

#include "nxbe.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_closewindow
 *
 * Description:
 *   Close an existing window
 *
 * Input Parameters:
 *   wnd  - The window to be closed (and deallocated using the user-space
 *          allocator)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_closewindow(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s *be;

  DEBUGASSERT(wnd != NULL);
  be = wnd->be;

  /* The background window should never be closed */

  DEBUGASSERT(wnd != &be->bkgd);

  /* Are we closing a modal window? */

  if (NXBE_ISMODAL(wnd))
    {
      /* Yes.. this should be the top window and the back-end should also
       * indicate the modal state.
       */

      DEBUGASSERT(wnd->above == NULL && NXBE_STATE_ISMODAL(be));

      /* Leave the modal state */

      NXBE_CLRMODAL(wnd);
      NXBE_STATE_CLRMODAL(be);
    }

  /* A hidden window does not exist in the hierarchy */

  if (!NXBE_ISHIDDEN(wnd))
    {
      /* Is there a window above the one being closed? */

      if (wnd->above != NULL)
        {
          /* Yes, now the window below that one is the window below
           * the one being closed.
           */

          wnd->above->below = wnd->below;
        }
      else
        {
          /* No, then the top window is the one below this (which
           * can never be NULL because the background window is
           * always at the true bottom of the list
           */

          be->topwnd = wnd->below;
        }

      /* There is always a window below the one being closed (because
       * the background is never closed.  Now, the window above that
       * is the window above the one that is being closed.
       */

      wnd->below->above = wnd->above;

      /* Redraw the windows that were below us (and may now be exposed) */

      nxbe_redrawbelow(be, wnd->below, &wnd->bounds);
    }

#ifdef CONFIG_NX_RAMBACKED
  /* Free any allocated, per-window framebuffer */

  if (wnd->fbmem != NULL)
    {
#ifdef CONFIG_BUILD_KERNEL
      DEBUGASSERT(wnd->npages > 0);

      /* Return the pages to the poll */

      mm_pgfree((uintptr_t)wnd->fbmem, wnd->npages);
#else
      /* Return the memory to the user heap */

      kumm_free(wnd->fbmem);
#endif
    }
#endif

  /* Then discard the window structure.  Here we assume that the user-space
   * allocator was used.
   */

  kumm_free(wnd);
}
