/****************************************************************************
 * libs/libnx/nxmu/nx_connect.c
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
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/mqueue.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#include "nxcontext.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Each client is assigned a unique ID using the g_nxcid counter.  That
 * counter increments as each new counter is created and is protected for
 * thread safety with g_nxlibsem.  Note that these are the only global values
 * in the NX implementation.  This is because the client ID must be unique
 * even across all server instances.
 *
 * NOTE: that client ID 0 is reserved for the server(s) themselves
 */

static mutex_t  g_nxliblock = NXMUTEX_INITIALIZER;
static uint32_t g_nxcid     = 1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_connectinstance
 *
 * Description:
 *   Open a connection from a client to the NX server.  One one client
 *   connection is normally needed per thread as each connection can host
 *   multiple windows.
 *
 *   NOTES:
 *   - This function returns before the connection is fully instantiated.
 *     it is necessary to wait for the connection event before using the
 *     returned handle.
 *   - Multiple instances of the NX server may run at the same time,
 *     each with different message queue names.
 *
 * Input Parameters:
 *   svrmqname - The name for the server incoming message queue
 *
 * Returned Value:
 *   Success: A non-NULL handle used with subsequent NX accesses
 *   Failure:  NULL is returned and errno is set appropriately
 *
 ****************************************************************************/

NXHANDLE nx_connectinstance(FAR const char *svrmqname)
{
  FAR struct nxmu_conn_s *conn;
  struct nxsvrmsg_s       outmsg;
  char                    climqname[NX_CLIENT_MXNAMELEN];
  struct mq_attr          attr;
  int                     ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!svrmqname)
    {
      set_errno(EINVAL);
      return NULL;
    }
#endif

  /* Allocate the NX client structure */

  conn = (FAR struct nxmu_conn_s *)lib_uzalloc(sizeof(struct nxmu_conn_s));
  if (!conn)
    {
      set_errno(ENOMEM);
      goto errout;
    }

  /* Create the client MQ name */

  nxmutex_lock(&g_nxliblock);
  conn->cid = g_nxcid++;
  nxmutex_unlock(&g_nxliblock);

  sprintf(climqname, NX_CLIENT_MQNAMEFMT, conn->cid);

  /* Open the client MQ for reading */

  attr.mq_maxmsg  = CONFIG_NX_MXCLIENTMSGS;
  attr.mq_msgsize = NX_MXCLIMSGLEN;
  attr.mq_flags   = 0;

#ifdef CONFIG_NX_BLOCKING
  conn->crdmq = _MQ_OPEN(climqname, O_RDONLY | O_CREAT, 0666, &attr);
#else
  conn->crdmq = _MQ_OPEN(climqname, O_RDONLY | O_CREAT | O_NONBLOCK,
                         0666, &attr);
#endif
  if (conn->crdmq < 0)
    {
      _NX_SETERRNO(conn->crdmq);
      gerr("ERROR: _MQ_OPEN(%s) failed: %d\n", climqname,
           _NX_GETERRNO(conn->crdmq));
      goto errout_with_conn;
    }

  /* Open the server MQ for writing */

  attr.mq_maxmsg  = CONFIG_NX_MXSERVERMSGS;
  attr.mq_msgsize = NX_MXSVRMSGLEN;
  attr.mq_flags   = 0;

  conn->cwrmq = _MQ_OPEN(svrmqname, O_WRONLY | O_CREAT, 0666, &attr);
  if (conn->cwrmq < 0)
    {
      _NX_SETERRNO(conn->cwrmq);
      gerr("ERROR: _MQ_OPEN(%s) failed: %d\n", svrmqname,
           _NX_GETERRNO(conn->crdmq));
      goto errout_with_rmq;
    }

  /* Inform the server that this client exists */

  outmsg.msgid = NX_SVRMSG_CONNECT;
  outmsg.conn  = conn;

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver failed: %d\n", get_errno());
      goto errout_with_wmq;
    }

#if 0
  /* Now read until we get a response to this message.  The server will
   * respond with either (1) NX_CLIMSG_CONNECTED, in which case the state
   * will change to NX_CLISTATE_CONNECTED, or (2) NX_CLIMSG_DISCONNECTED
   * in which case, nx_message will fail with errno = EHOSTDOWN.
   */

  do
    {
      ret = nx_eventhandler((NXHANDLE)conn);
      if (ret < 0)
        {
          gerr("ERROR: nx_message failed: %d\n", get_errno());
          goto errout_with_wmq;
        }

      _SIG_USLEEP(300000);
    }
  while (conn->state != NX_CLISTATE_CONNECTED);
#endif
  return (NXHANDLE)conn;

errout_with_wmq:
  _MQ_CLOSE(conn->cwrmq);
errout_with_rmq:
  _MQ_CLOSE(conn->crdmq);
errout_with_conn:
  lib_ufree(conn);
errout:
  return NULL;
}
