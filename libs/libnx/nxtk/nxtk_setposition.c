/****************************************************************************
 * libs/libnx/nxtk/nxtk_setposition.c
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
#include <nuttx/nx/nxtk.h>

#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_setposition
 *
 * Description:
 *  Set the position for the selected client window.  This position does not
 *  include the offsets for the borders nor for any toolbar.  Those offsets
 *  will be added in to set the full window position.
 *
 * Input Parameters:
 *   hfwnd - The window handle returned by nxtk_openwindow
 *   pos   - The new position of the client sub-window
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_setposition(NXTKWINDOW hfwnd, FAR const struct nxgl_point_s *pos)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                             (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_point_s offset;
  struct nxgl_point_s newpos;

  /* Calculate the offset that is requested and add that to the window
   * origin.
   */

  nxgl_vectsubtract(&offset, pos, &fwnd->fwrect.pt1);
  nxgl_vectoradd(&newpos, &offset, &fwnd->wnd.bounds.pt1);

  /* Then set that position */

  return nx_setposition((NXWINDOW)hfwnd, &newpos);
}
