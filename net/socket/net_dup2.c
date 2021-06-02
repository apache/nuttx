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
#ifdef NET_TCP_HAVE_STACK
  FAR struct tcp_conn_s *conn;
#endif
  int ret = OK;

  /* Parts of this operation need to be atomic */

  net_lock();

  /* Duplicate the relevant socket state (zeroing everything else) */

  memset(psock2, 0, sizeof(struct socket));

  psock2->s_domain   = psock1->s_domain;    /* IP domain: PF_INET, PF_INET6, or PF_PACKET */
  psock2->s_type     = psock1->s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  psock2->s_sockif   = psock1->s_sockif;    /* Socket interface */
  psock2->s_flags    = psock1->s_flags;     /* See _SF_* definitions */
#ifdef CONFIG_NET_SOCKOPTS
  psock2->s_options  = psock1->s_options;   /* Selected socket options */
  psock2->s_rcvtimeo = psock1->s_rcvtimeo;  /* Receive timeout value (in deciseconds) */
  psock2->s_sndtimeo = psock1->s_sndtimeo;  /* Send timeout value (in deciseconds) */
#ifdef CONFIG_NET_SOLINGER
  psock2->s_linger   = psock1->s_linger;    /* Linger timeout value (in deciseconds) */
#endif
#endif
  psock2->s_conn     = psock1->s_conn;      /* UDP or TCP connection structure */

  /* Increment the reference count on the underlying connection structure
   * for this address family type.
   */

  DEBUGASSERT(psock2->s_sockif != NULL &&
              psock2->s_sockif->si_addref != NULL);
  psock2->s_sockif->si_addref(psock2);

#ifdef NET_TCP_HAVE_STACK
  /* For connected socket types, it is necessary to also start the network
   * monitor so that the newly cloned socket can receive a notification if
   * the network connection is lost.
   */

  conn = (FAR struct tcp_conn_s *)psock2->s_conn;

  if (psock2->s_type == SOCK_STREAM && conn &&
      (conn->tcpstateflags == TCP_ESTABLISHED ||
       conn->tcpstateflags == TCP_SYN_RCVD))
    {
      ret = tcp_start_monitor(psock2);

      /* On failure, back out the reference count on the TCP connection
       * structure.  tcp_start_monitor() will fail only in the race condition
       * where the TCP connection has been lost.
       */

      if (ret < 0)
        {
          /* There should be at least two reference counts on the connection
           * structure:  At least one from the original socket and the one
           * from above where we incremented the reference count.
           * inet_close() will handle all cases.
           *
           * NOTE:  As a side-effect, inet_close()will also call
           * tcp_stop_monitor() which could inform the loss of connection to
           * all open sockets on the connection structure if the reference
           * count decrements to zero.
           */

          inet_close(psock2);

          /* The socket will not persist... reset it */

          memset(psock2, 0, sizeof(*psock2));
        }
    }
#endif

  net_unlock();
  return ret;
}
