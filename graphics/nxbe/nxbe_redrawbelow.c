/****************************************************************************
 * graphics/nxbe/nxbe_redrawbelow.c
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

#include "nxbe.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_redrawbelow
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for all windows
 *   below (and including) the specified window.  This function is called
 *   whenever a window is closed, moved, lowered or re-sized in order to
 *   expose newly visible portions of lower windows.
 *
 ****************************************************************************/

void nxbe_redrawbelow(FAR struct nxbe_state_s *be,
                      FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *currwnd;

  for (currwnd = wnd; currwnd; currwnd = currwnd->below)
    {
      nxbe_redraw(be, currwnd, rect);
    }
}
