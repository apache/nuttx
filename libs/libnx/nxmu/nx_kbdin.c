/****************************************************************************
 * libs/libnx/nxmu/nx_kbdin.c
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

#include <stdint.h>
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#include "nxcontext.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_kbdin
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of keypad
 *   hardware to report text information to the NX server.  That text
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

int nx_kbdin(NXHANDLE handle, uint8_t nch, FAR const uint8_t *ch)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  FAR struct nxsvrmsg_kbdin_s *outmsg;
  int size;
  int ret;
  int i;

  /* Allocate a bigger message to account for the variable amount of
   * character data.
   */

  size = sizeof(struct nxsvrmsg_kbdin_s) + nch - 1;
  outmsg = (FAR struct nxsvrmsg_kbdin_s *)lib_malloc(size);
  if (!outmsg)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  /* Inform the server of the new keypad data */

  outmsg->msgid = NX_SVRMSG_KBDIN;
  outmsg->nch   = nch;

  for (i = 0; i < nch; i++)
    {
      outmsg->ch[i] = ch[i];
    }

  ret = nxmu_sendserver(conn, outmsg, size);

  lib_free(outmsg);
  return ret;
}

#endif /* CONFIG_NX_KBD */
