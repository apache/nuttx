/****************************************************************************
 * libs/libnx/nxmu/nx_connect.c
 *
 *   Copyright (C) 2008-2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/signal.h>
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

static sem_t    g_nxlibsem =  SEM_INITIALIZER(1);
static uint32_t g_nxcid    = 1;

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

  nxmu_semtake(&g_nxlibsem);
  conn->cid = g_nxcid++;
  nxmu_semgive(&g_nxlibsem);

  sprintf(climqname, NX_CLIENT_MQNAMEFMT, conn->cid);

  /* Open the client MQ for reading */

  attr.mq_maxmsg  = CONFIG_NX_MXCLIENTMSGS;
  attr.mq_msgsize = NX_MXCLIMSGLEN;
  attr.mq_flags   = 0;

#ifdef CONFIG_NX_BLOCKING
  conn->crdmq = mq_open(climqname, O_RDONLY|O_CREAT, 0666, &attr);
#else
  conn->crdmq = mq_open(climqname, O_RDONLY|O_CREAT|O_NONBLOCK, 0666, &attr);
#endif
  if (conn->crdmq == (mqd_t)-1)
    {
      gerr("ERROR: mq_open(%s) failed: %d\n", climqname, errno);
      goto errout_with_conn;
    }

  /* Open the server MQ for writing */

  attr.mq_maxmsg  = CONFIG_NX_MXSERVERMSGS;
  attr.mq_msgsize = NX_MXSVRMSGLEN;
  attr.mq_flags   = 0;

  conn->cwrmq = mq_open(svrmqname, O_WRONLY|O_CREAT, 0666, &attr);
  if (conn->cwrmq == (mqd_t)-1)
    {
      gerr("ERROR: mq_open(%s) failed: %d\n", svrmqname, errno);
      goto errout_with_rmq;
    }

  /* Inform the server that this client exists */

  outmsg.msgid = NX_SVRMSG_CONNECT;
  outmsg.conn  = conn;

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver failed: %d\n", errno);
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
          gerr("ERROR: nx_message failed: %d\n", errno);
          goto errout_with_wmq;
        }

      _SIG_USLEEP(300000);
    }
  while (conn->state != NX_CLISTATE_CONNECTED);
#endif
  return (NXHANDLE)conn;

errout_with_wmq:
  mq_close(conn->cwrmq);
errout_with_rmq:
  mq_close(conn->crdmq);
errout_with_conn:
  lib_ufree(conn);
errout:
  return NULL;
}
