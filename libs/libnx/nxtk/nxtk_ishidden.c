/****************************************************************************
 * libs/libnx/nxtk/nxtk_ishidden.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxtk.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_ishidden
 *
 * Description:
 *   Return true if the window is hidden.
 *
 *   NOTE:  There will be a delay between the time that the visibility of
 *   the window is changed via nxtk_setvisibily() before that new setting is
 *   reported by nxtk_ishidden().  nxtk_synch() may be used if temporal
 *   synchronization is required.
 *
 * Input Parameters:
 *   hfwnd - The window to be queried
 *
 * Returned Value:
 *   True: the window is hidden, false: the window is visible
 *
 ****************************************************************************/

bool nxtk_ishidden(NXTKWINDOW hfwnd)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hfwnd;
  return NXBE_ISHIDDEN(wnd);
}
