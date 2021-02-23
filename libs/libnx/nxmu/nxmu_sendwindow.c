/****************************************************************************
 * libs/libnx/nxmu/nxmu_sendwindow.c
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

#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_sendwindow
 *
 * Description:
 *  Send a message to the server destined for a specific window at
 *  NX_SVRMSG_PRIO priority
 *
 * Input Parameters:
 *   wnd    - A pointer to the back-end window structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendwindow(FAR struct nxbe_window_s *wnd, FAR const void *msg,
                    size_t msglen)
{
  int ret = OK;

  /* Sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!wnd || !wnd->conn)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Ignore messages destined to a blocked window (no errors reported) */

  if (!NXBE_ISBLOCKED(wnd))
    {
      /* Send the message to the server */

      ret = nxmu_sendserver(wnd->conn, msg, msglen);
    }

  return ret;
}
