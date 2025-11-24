/****************************************************************************
 * net/pkt/pkt_sockif.c
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
#include <sys/socket.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netpacket/packet.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "utils/utils.h"
#include <socket/socket.h>
#include "pkt/pkt.h"

#ifdef CONFIG_NET_PKT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        pkt_setup(FAR struct socket *psock);
static sockcaps_t pkt_sockcaps(FAR struct socket *psock);
static void       pkt_addref(FAR struct socket *psock);
static int        pkt_bind(FAR struct socket *psock,
                           FAR const struct sockaddr *addr,
                           socklen_t addrlen);
static int        pkt_netpoll(FAR struct socket *psock,
                              FAR struct pollfd *fds, bool setup);
static int        pkt_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_pkt_sockif =
{
  pkt_setup,       /* si_setup */
  pkt_sockcaps,    /* si_sockcaps */
  pkt_addref,      /* si_addref */
  pkt_bind,        /* si_bind */
  NULL,            /* si_getsockname */
  NULL,            /* si_getpeername */
  NULL,            /* si_listen */
  NULL,            /* si_connect */
  NULL,            /* si_accept */
  pkt_netpoll,     /* si_poll */
  pkt_sendmsg,     /* si_sendmsg */
  pkt_recvmsg,     /* si_recvmsg */
  pkt_close,       /* si_close */
  NULL,            /* si_ioctl */
  NULL,            /* si_socketpair */
  NULL             /* si_shutdown */
#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_PKTPROTO_OPTIONS)
  , pkt_getsockopt /* si_getsockopt */
  , pkt_setsockopt /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_sockif_alloc
 *
 * Description:
 *   Allocate and attach a raw packet connection structure.
 *
 ****************************************************************************/

static int pkt_sockif_alloc(FAR struct socket *psock)
{
  /* Allocate the packet socket connection structure and save in the new
   * socket instance.
   */

  FAR struct pkt_conn_s *conn = pkt_alloc();
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the protocol in the connection structure */

  conn->type = psock->s_proto;

#ifdef CONFIG_NET_PKT_WRITE_BUFFERS
#  if CONFIG_NET_SEND_BUFSIZE > 0
  conn->sndbufs = CONFIG_NET_SEND_BUFSIZE;
#  endif
  nxsem_init(&conn->sndsem, 0, 0);
#endif

  nxrmutex_init(&conn->sconn.s_lock);

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}

/****************************************************************************
 * Name: pkt_setup
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

static int pkt_setup(FAR struct socket *psock)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * connection structure, it is unallocated at this point.  It will not
   * actually be initialized until the socket is connected.
   *
   * SOCK_RAW is supported.
   */

  if (psock->s_type == SOCK_DGRAM || psock->s_type == SOCK_RAW)
    {
      return pkt_sockif_alloc(psock);
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: pkt_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The set of socket capabilities is returned.
 *
 ****************************************************************************/

static sockcaps_t pkt_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: pkt_addref
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

static void pkt_addref(FAR struct socket *psock)
{
  FAR struct pkt_conn_s *conn;

  DEBUGASSERT(psock->s_type == SOCK_RAW);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: pkt_bind
 *
 * Description:
 *   pkt_bind() gives the socket 'psock' the local address 'addr'.  'addr'
 *   is 'addrlen' bytes long.  Traditionally, this is called "assigning a
 *   name to a socket."  When a socket is created with socket(), it exists
 *   in a name space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int pkt_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen)
{
  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_PACKET || addrlen < sizeof(struct sockaddr_ll))
    {
      nerr("ERROR: Invalid address length: %d < %zu\n",
           addrlen, sizeof(struct sockaddr_ll));
      return -EBADF;
    }

  /* Bind a raw socket to a network device. */

  if (psock->s_type == SOCK_DGRAM || psock->s_type == SOCK_RAW)
    {
      FAR struct pkt_conn_s *conn = psock->s_conn;
      FAR struct net_driver_s *dev;

      /* Look at the addr and identify the network interface */

      int ifindex = ((FAR struct sockaddr_ll *)addr)->sll_ifindex;
      int protocol = ((FAR struct sockaddr_ll *)addr)->sll_protocol;

      /* Check if we have that interface */

      dev = netdev_findbyindex(ifindex);
      if (dev == NULL)
        {
          return -EADDRNOTAVAIL;
        }

      /* Put ifindex into connection */

      conn->ifindex = ifindex;
      if (protocol != 0)
        {
          conn->type = protocol;
        }

      return OK;
    }
  else
    {
      return -EBADF;
    }
}

/****************************************************************************
 * Name: pkt_netpoll
 *
 * Description:
 *   The standard poll() operation redirects operations on pkt socket
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

static int pkt_netpoll(FAR struct socket *psock, FAR struct pollfd *fds,
                       bool setup)
{
  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the PKT poll() setup */

      return pkt_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the PKT poll() teardown */

      return pkt_pollteardown(psock, fds);
    }
}

/****************************************************************************
 * Name: pkt_close
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

static int pkt_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the raw packet address type */

  switch (psock->s_type)
    {
      case SOCK_DGRAM:
      case SOCK_RAW:
        {
          FAR struct pkt_conn_s *conn = psock->s_conn;
          FAR struct net_driver_s *dev = pkt_find_device(conn);

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->crefs <= 1)
            {
              conn_dev_lock(&conn->sconn, dev);

              /* Yes... free any read-ahead data */

              iob_free_queue(&conn->readahead);

#ifdef CONFIG_NET_PKT_WRITE_BUFFERS
              /* Free write buffer callback. */

              if (conn->sndcb != NULL)
                {
                  int ret;

                  while (iob_get_queue_entry_count(&conn->write_q) != 0)
                    {
                      ret = conn_dev_sem_timedwait(&conn->sndsem, false,
                                         _SO_TIMEOUT(conn->sconn.s_sndtimeo),
                                         &conn->sconn, dev);
                      if (ret < 0)
                        {
                          break;
                        }
                    }

                  pkt_callback_free(dev, conn, conn->sndcb);
                  conn->sndcb = NULL;
                }
#endif

              /* Then free the connection structure */

              conn->crefs = 0;          /* No more references on the connection */
              conn_dev_unlock(&conn->sconn, dev);
              pkt_free(psock->s_conn);  /* Free network resources */
            }
          else
            {
              /* No.. Just decrement the reference count */

              conn->crefs--;
            }

          return OK;
        }

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_PKT */
