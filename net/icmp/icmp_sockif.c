/****************************************************************************
 * net/icmp/icmp_sockif.c
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
#include <sys/socket.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <socket/socket.h>

#include "icmp/icmp.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        icmp_setup(FAR struct socket *psock);
static sockcaps_t icmp_sockcaps(FAR struct socket *psock);
static void       icmp_addref(FAR struct socket *psock);
static int        icmp_netpoll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static int        icmp_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_icmp_sockif =
{
  icmp_setup,       /* si_setup */
  icmp_sockcaps,    /* si_sockcaps */
  icmp_addref,      /* si_addref */
  NULL,             /* si_bind */
  NULL,             /* si_getsockname */
  NULL,             /* si_getpeername */
  NULL,             /* si_listen */
  NULL,             /* si_connect */
  NULL,             /* si_accept */
  icmp_netpoll,     /* si_poll */
  icmp_sendmsg,     /* si_sendmsg */
  icmp_recvmsg,     /* si_recvmsg */
  icmp_close,       /* si_close */
  icmp_ioctl        /* si_ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int icmp_setup(FAR struct socket *psock)
{
  /* SOCK_DGRAM or SOCK_CTRL and IPPROTO_ICMP are supported */

  if ((psock->s_type == SOCK_DGRAM || psock->s_type == SOCK_CTRL) &&
       psock->s_proto == IPPROTO_ICMP)
    {
      /* Allocate the IPPROTO_ICMP socket connection structure and save in
       * the new socket instance.
       */

      FAR struct icmp_conn_s *conn = icmp_alloc();
      if (conn == NULL)
        {
          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

      /* Set the reference count on the connection structure.
       * This reference count will be incremented only if the socket is
       * dup'ed
       */

      DEBUGASSERT(conn->crefs == 0);
      conn->crefs = 1;

      /* Save the pre-allocated connection in the socket structure */

      psock->s_conn = conn;
      return OK;
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: icmp_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t icmp_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: icmp_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void icmp_addref(FAR struct socket *psock)
{
  FAR struct icmp_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: icmp_netpoll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to net_poll which, indiectly, calls to function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int icmp_netpoll(FAR struct socket *psock, FAR struct pollfd *fds,
                        bool setup)
{
  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the ICMP poll() setup */

      return icmp_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the ICMP poll() teardown */

      return icmp_pollteardown(psock, fds);
    }
}

/****************************************************************************
 * Name: icmp_close
 *
 * Description:
 *   Performs the close operation on a raw packet socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; a negated errno value is returned on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int icmp_close(FAR struct socket *psock)
{
  FAR struct icmp_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = psock->s_conn;

  /* Is this the last reference to the connection structure (there could be\
   * more if the socket was dup'ed).
   */

  DEBUGASSERT(conn->crefs > 0);

  if (conn->crefs <= 1)
    {
      /* Yes... free any read-ahead data */

      iob_free_queue(&conn->readahead);

      /* Then free the connection structure */

      conn->crefs = 0;           /* No more references on the connection */
      icmp_free(psock->s_conn);  /* Free network resources */
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_ICMP_SOCKET */
