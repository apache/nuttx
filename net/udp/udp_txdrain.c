/****************************************************************************
 * net/udp/udp_txdrain.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>
#include <nuttx/semaphore.h>
#include <nuttx/tls.h>

#include "utils/utils.h"
#include "udp/udp.h"

#if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_NET_UDP_NOTIFIER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_txdrain
 *
 * Description:
 *   Wait for all write buffers to be sent (or for a timeout to occur).
 *
 * Input Parameters:
 *   psock   - An instance of the internal socket structure.
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int udp_txdrain(FAR struct socket *psock, unsigned int timeout)
{
  FAR struct udp_conn_s *conn;
  sem_t waitsem;
  int ret = OK;

  DEBUGASSERT(psock->s_type == SOCK_DGRAM);

  /* udp_txdrain() is a cancellation point */

  enter_cancellation_point();

  conn = psock->s_conn;

  /* Initialize the wait semaphore */

  nxsem_init(&waitsem, 0, 0);

  /* The following needs to be done with the network stable */

  conn_lock(&conn->sconn);
  if (!sq_empty(&conn->write_q))
    {
      conn->txdrain_sem = &waitsem;
      ret = conn_dev_sem_timedwait(&waitsem, false, timeout,
                                   &conn->sconn, NULL);
      conn->txdrain_sem = NULL;
    }

  conn_unlock(&conn->sconn);
  nxsem_destroy(&waitsem);
  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET_UDP_WRITE_BUFFERS && CONFIG_NET_UDP_NOTIFIER */
