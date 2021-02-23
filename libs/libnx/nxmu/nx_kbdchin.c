/****************************************************************************
 * libs/libnx/nxmu/nx_kbdchin.c
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
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_kbdchin
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of keypad
 *   hardware to report text information to the NX server.  That text
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

int nx_kbdchin(NXHANDLE handle, uint8_t ch)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_kbdin_s outmsg;

  /* Inform the server of the new keypad data */

  outmsg.msgid = NX_SVRMSG_KBDIN;
  outmsg.nch   = 1;
  outmsg.ch[0] = ch;

  return nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_kbdin_s));
}

#endif /* CONFIG_NX_KBD */
