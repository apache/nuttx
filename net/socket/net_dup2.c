/****************************************************************************
 * net/socket/net_dup2.c
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

#include <sys/socket.h>
#include <string.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>

#include "inet/inet.h"
#include "tcp/tcp.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_dup2
 *
 * Description:
 *   Performs the low level, common portion of dup
 *
 * Input Parameters:
 *   psock1 - The existing socket that is being cloned.
 *   psock2 - A reference to an uninitialized socket structure allocated by
 *            the caller.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int psock_dup2(FAR struct socket *psock1, FAR struct socket *psock2)
{
  /* Parts of this operation need to be atomic */

  net_lock();

  /* Duplicate the relevant socket state (zeroing everything else) */

  memset(psock2, 0, sizeof(struct socket));

  psock2->s_domain   = psock1->s_domain;    /* IP domain: PF_INET, PF_INET6, or PF_PACKET */
  psock2->s_type     = psock1->s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  psock2->s_sockif   = psock1->s_sockif;    /* Socket interface */
  psock2->s_conn     = psock1->s_conn;      /* UDP or TCP connection structure */

  /* Increment the reference count on the underlying connection structure
   * for this address family type.
   */

  DEBUGASSERT(psock2->s_sockif != NULL &&
              psock2->s_sockif->si_addref != NULL);
  psock2->s_sockif->si_addref(psock2);

  net_unlock();

  return OK;
}
