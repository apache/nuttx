/****************************************************************************
 * libs/libnx/nxmu/nx_synch.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_synch
 *
 * Description:
 *   This interface can be used to syncrhonize the window client with the
 *   NX server.  It really just implements an 'echo':  A synch message is
 *   sent from the window client to the server which then responds
 *   immediately by sending the NXEVENT_SYNCHED back to the windows client.
 *
 *   Due to the highly asynchronous nature of client-server communications,
 *   nx_synch() is sometimes necessary to assure that the client and server
 *   are fully synchronized in time.
 *
 *   Usage by the window client might be something like this:
 *
 *     extern bool g_synched;
 *     extern sem_t g_synch_sem;
 *
 *     g_synched = false;
 *     ret = nx_synch(hwnd, handle);
 *     if (ret < 0)
 *       {
 *          -- Handle the error --
 *       }
 *
 *     while (!g_synched)
 *       {
 *         ret = sem_wait(&g_sync_sem);
 *         if (ret < 0)
 *           {
 *              -- Handle the error --
 *           }
 *       }
 *
 *   When the windwo listener thread receives the NXEVENT_SYNCHED event, it
 *   would set g_synched to true and post g_synch_sem, waking up the above
 *   loop.
 *
 * Input Parameters:
 *   wnd - The window to be synched
 *   arg - An argument that will accompany the block messages (This is arg2
 *         in the event callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nx_synch(NXWINDOW hwnd, FAR void *arg)
{
  struct nxsvrmsg_synch_s outmsg;

#ifdef CONFIG_DEBUG_FEATURES
  if (hwnd == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Send the syncrhonization request message. */

  outmsg.msgid = NX_SVRMSG_SYNCH;
  outmsg.wnd   = (FAR struct nxbe_window_s *)hwnd;
  outmsg.arg   = arg;

  return nxmu_sendwindow(hwnd, &outmsg, sizeof(struct nxsvrmsg_synch_s));
}
