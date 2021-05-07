/****************************************************************************
 * libs/libnx/nxmu/nx_disconnect.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mqueue.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_disconnect
 *
 * Description:
 *   Disconnect a client from the NX server and/or free resources reserved
 *   by nx_connect/nx_connectinstance.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Returned Value:
 *   OK on success; ERROR on failure with the errno set appropriately.
 *   NOTE that handle will no long be valid upon return.
 *
 ****************************************************************************/

void nx_disconnect(NXHANDLE handle)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_s       outmsg;
  char                    climqname[NX_CLIENT_MXNAMELEN];
  int                     ret;

  /* Inform the server that this client no longer exists */

  outmsg.msgid = NX_SVRMSG_DISCONNECT;
  outmsg.conn  = conn;

  /* We will finish the teardown upon receipt of the DISCONNECTED message */

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver() returned %d\n", ret);
    }
  else
    {
      snprintf(climqname, sizeof(climqname),
               NX_CLIENT_MQNAMEFMT, conn->cid);
      _MQ_UNLINK(climqname);
    }
}
