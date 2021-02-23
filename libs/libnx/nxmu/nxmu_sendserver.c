/****************************************************************************
 * libs/libnx/nxmu/nxmu_sendserver.c
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

#include <nuttx/mqueue.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_sendserver
 *
 * Description:
 *  Send a message to the server at NX_SVRMSG_PRIO priority
 *
 * Input Parameters:
 *   conn   - A pointer to the server connection structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendserver(FAR struct nxmu_conn_s *conn, FAR const void *msg,
                    size_t msglen)
{
  int ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn || !conn->cwrmq)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Send the message to the server */

  ret = _MQ_SEND(conn->cwrmq, msg, msglen, NX_SVRMSG_PRIO);
  if (ret < 0)
    {
      _NX_SETERRNO(ret);
      gerr("ERROR: _MQ_SEND failed: %d\n", _NX_GETERRNO(ret));
      ret = ERROR;
    }

  return ret;
}
