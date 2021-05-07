/****************************************************************************
 * libs/libnx/nxtk/nxtk_block.c
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
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_block
 *
 * Description:
 *   This is callback will do to things:  (1) any queue a 'blocked' callback
 *   to the window and then (2) block any further window messaging.
 *
 *   The 'blocked' callback is the response from nx_block (or nxtk_block).
 *   Those blocking interfaces are used to assure that no further messages
 *   are are directed to the window. Receipt of the blocked callback
 *   signifies that (1) there are no further pending callbacks and (2) that
 *   the window is now 'defunct' and will receive no further callbacks.
 *
 *   This callback supports coordinated destruction of a window in multi-
 *   user mode.  In multi-use mode, the client window logic must stay
 *   intact until all of the queued callbacks are processed.  Then the
 *   window may be safely closed.  Closing the window prior with pending
 *   callbacks can lead to bad behavior when the callback is executed.
 *
 * Input Parameters:
 *   hfwnd - The window to be blocked
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the blocked callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_block(NXTKWINDOW hfwnd, FAR void *arg)
{
  return nx_block((NXWINDOW)hfwnd, arg);
}
