/****************************************************************************
 * graphics/nxbe/nxbe_closewindow.c
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
