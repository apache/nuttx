/****************************************************************************
 * libs/libnx/nxmu/nx_block.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_block
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
 *   wnd - The window to be blocked
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the blocked callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_block(NXWINDOW hwnd, FAR void *arg)
{
  FAR struct nxbe_window_s *wnd = (FAR struct nxbe_window_s *)hwnd;
  struct nxsvrmsg_blocked_s outmsg;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hwnd)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Ignore additional attempts to block messages (no errors reported) */

  if (!NXBE_ISBLOCKED(wnd))
    {
      /* Mark the window as blocked.  This will stop all messages to the
       * window (EXCEPT the NX_SVRMSG_BLOCKED).
       * Blocking the messages before sending the blocked message
       * is awkward but assures that no other messages sneak into
       * the message queue before we can set the blocked state.
       */

      NXBE_SETBLOCKED(wnd);

      /* Send the message indicating that the window is blocked
       * (and because of queue also that there are no additional queue
       * messages for the window)
       */

      outmsg.msgid = NX_SVRMSG_BLOCKED;
      outmsg.wnd   = wnd;
      outmsg.arg   = arg;

      /* Send the window message via nxmu_sendserver (vs. nxmu_sendwindow) so
       * that it will not be blocked.
       */

      ret = nxmu_sendserver(wnd->conn, &outmsg,
                            sizeof(struct nxsvrmsg_blocked_s));
    }

  return ret;
}
