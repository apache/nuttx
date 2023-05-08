/****************************************************************************
 * net/tcp/tcp_accept.c
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
#ifdef CONFIG_NET_TCP

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accept_s
{
  sem_t                  acpt_sem;        /* Wait for driver event */
  FAR struct sockaddr   *acpt_addr;       /* Return connection address */
  FAR socklen_t         *acpt_addrlen;    /* Return length of address */
  FAR struct tcp_conn_s *acpt_newconn;    /* The accepted connection */
  int                    acpt_result;     /* The result of the wait */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accept_tcpsender
 *
 * Description:
 *   Get the sender's address from the UDP packet
 *
 * Input Parameters:
 *   listener - The connection structure of the listener
 *   conn     - The newly accepted TCP connection
 *   pstate   - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static inline void accept_tcpsender(FAR struct tcp_conn_s *listener,
                                    FAR struct tcp_conn_s *conn,
                                    FAR struct sockaddr *addr,
                                    socklen_t *addrlen)
{
  if (addr)
    {
      /* If an address is provided, then the length must also be provided. */

      DEBUGASSERT(addrlen);

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      /* If both IPv4 and IPv6 support are enabled, then we will need to
       * select which one to use when obtaining the sender's IP address.
       */

      if (listener->domain == PF_INET)
#endif /* CONFIG_NET_IPv6 */
        {
          FAR struct sockaddr_in *inaddr = (FAR struct sockaddr_in *)addr;

          inaddr->sin_family = AF_INET;
          inaddr->sin_port   = conn->rport;
          net_ipv4addr_copy(inaddr->sin_addr.s_addr, conn->u.ipv4.raddr);
          memset(inaddr->sin_zero, 0, sizeof(inaddr->sin_zero));

          *addrlen = sizeof(struct sockaddr_in);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      /* Otherwise, this the IPv6 address is needed */

      else
#endif /* CONFIG_NET_IPv4 */
        {
          FAR struct sockaddr_in6 *inaddr = (FAR struct sockaddr_in6 *)addr;

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          DEBUGASSERT(listener->domain == PF_INET6);
#endif
          inaddr->sin6_family = AF_INET6;
          inaddr->sin6_port   = conn->rport;
          net_ipv6addr_copy(inaddr->sin6_addr.s6_addr, conn->u.ipv6.raddr);

          *addrlen = sizeof(struct sockaddr_in6);
        }
#endif /* CONFIG_NET_IPv6 */
    }
}

/****************************************************************************
 * Name: accept_eventhandler
 *
 * Description:
 *   Receive event callbacks when connections occur
 *
 * Input Parameters:
 *   listener The connection structure of the listener
 *   conn     The connection structure that was just accepted
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static int accept_eventhandler(FAR struct tcp_conn_s *listener,
                               FAR struct tcp_conn_s *conn)
{
  struct accept_s *pstate = (struct accept_s *)listener->accept_private;
  int ret = -EINVAL;

  if (pstate)
    {
      /* Get the connection address */

      accept_tcpsender(listener, conn, pstate->acpt_addr,
                       pstate->acpt_addrlen);

      /* Save the connection structure */

      pstate->acpt_newconn     = conn;
      pstate->acpt_result      = OK;

      /* There should be a reference of one on the new connection */

      DEBUGASSERT(conn->crefs == 1);

      /* Wake-up the waiting caller thread */

      nxsem_post(&pstate->acpt_sem);

      /* Stop any further callbacks */

      listener->accept_private = NULL;
      listener->accept         = NULL;
      ret                      = OK;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_accept
 *
 * Description:
 *   This function implements accept() for TCP/IP sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening TCP socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of
 *            'addr'
 *   newconn  The new, accepted TCP connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is locked.
 *
 ****************************************************************************/

int psock_tcp_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen, FAR void **newconn)
{
  FAR struct tcp_conn_s *conn;
  struct accept_s state;
  int ret;

  DEBUGASSERT(psock && newconn);

  /* Check the backlog to see if there is a connection already pending for
   * this listener.
   */

  conn = psock->s_conn;

#ifdef CONFIG_NET_TCPBACKLOG
  state.acpt_newconn = tcp_backlogremove(conn);
  if (state.acpt_newconn)
    {
      /* Yes... get the address of the connected client */

      ninfo("Pending conn=%p\n", state.acpt_newconn);
      accept_tcpsender(conn, state.acpt_newconn, addr, addrlen);
    }

  /* In general, this implementation will not support non-blocking socket
   * operations... except in a few cases:  Here for TCP accept with
   * backlog enabled.  If this socket is configured as non-blocking then
   * return EAGAIN if there is no pending connection in the backlog.
   */

  else if (_SS_ISNONBLOCK(conn->sconn.s_flags))
    {
      return -EAGAIN;
    }
  else
#endif
    {
      /* Perform the TCP accept operation */

      /* Initialize the state structure.  This is done with the network
       * locked because we don't want anything to happen until we are
       * ready.
       */

      state.acpt_addr       = addr;
      state.acpt_addrlen    = addrlen;
      state.acpt_newconn    = NULL;
      state.acpt_result     = OK;

      nxsem_init(&state.acpt_sem, 0, 0);

      /* Set up the callback in the connection */

      conn->accept_private  = (FAR void *)&state;
      conn->accept          = accept_eventhandler;

      /* Wait for the send to complete or an error to occur:  NOTES:
       * net_sem_wait will also terminate if a signal is received.
       */

      ret = net_sem_wait(&state.acpt_sem);

      /* Make sure that no further events are processed */

      conn->accept_private = NULL;
      conn->accept         = NULL;

      nxsem_destroy(&state.acpt_sem);

      /* Check for a errors.  Errors are signalled by negative errno values
       * for the send length.
       */

      if (state.acpt_result != 0)
        {
          DEBUGASSERT(state.acpt_result > 0);
          return -state.acpt_result;
        }

      /* If net_sem_wait failed, then we were probably reawakened by a
       * signal.  In this case, net_sem_wait will have returned negated
       * errno appropriately.
       */

      if (ret < 0)
        {
          return ret;
        }
    }

  *newconn = (FAR void *)state.acpt_newconn;
  return OK;
}

#endif /* CONFIG_NET_TCP */
