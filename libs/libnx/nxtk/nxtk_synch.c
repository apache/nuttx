/****************************************************************************
 * libs/libnx/nxtk/nxtk_synch.c
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
 * Name: nxtk_synch
 *
 * Description:
 *   This interface can be used to synchronize the window client with the
 *   NX server.  It really just implements an 'echo':  A synch message is
 *   sent from the window client to the server which then responds
 *   immediately by sending the NXEVENT_SYNCHED back to the windows client.
 *
 *   Due to the highly asynchronous nature of client-server communications,
 *   nxtk_synch() is sometimes necessary to assure that the client and server
 *   are fully synchronized in time.
 *
 *   Usage by the window client might be something like this:
 *
 *     extern bool g_synched;
 *     extern sem_t g_synch_sem;
 *
 *     g_synched = false;
 *     ret = nxtk_synch(hwnd, handle);
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
 *   When the window listener thread receives the NXEVENT_SYNCHED event, it
 *   would set g_synched to true and post g_synch_sem, waking up the above
 *   loop.
 *
 * Input Parameters:
 *   hfwnd - The window to be synched
 *   arg   - An argument that will accompany the block messages (This is arg2
 *           in the event callback).
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_synch(NXTKWINDOW hfwnd, FAR void *arg)
{
  return nx_synch((NXWINDOW)hfwnd, arg);
}
