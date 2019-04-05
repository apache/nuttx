/****************************************************************************
 * graphics/nxbe/nxbe_modal.c
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

#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 ****************************************************************************/

void nxbe_modal(FAR struct nxbe_window_s *wnd, bool enable)
{
  FAR struct nxbe_state_s *be = wnd->be;

  /* Are we enabling or disabling the modal state? */

  if (enable)
    {
      /* We are enabling the modal state.  Ignore the request if the back-end
       * is already in a modal state.
       */

      if (!NXBE_STATE_ISMODAL(be))
        {
          /* Raise the window to the top of the display */

          void nxbe_raise(FAR struct nxbe_window_s *wnd);

          /* And enter the modal state */

          DEBUGASSERT(!NXBE_ISMODAL(wnd));
          NXBE_SETMODAL(wnd);
          NXBE_STATE_SETMODAL(be);
        }
    }

  /* We are disabling the modal state.  Verify that we are in a modal state */

  else if (NXBE_STATE_ISMODAL(be) && NXBE_ISMODAL(wnd))
    {
      /* We are in a modal state and this window is the modal window.
       * Just disable the modal state.
       */

      NXBE_CLRMODAL(wnd);
      NXBE_STATE_CLRMODAL(be);
    }
}
