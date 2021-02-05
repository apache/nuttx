/****************************************************************************
 * graphics/nxbe/nxbe_modal.c
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
