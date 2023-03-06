/****************************************************************************
 * net/pkt/pkt_sockif.c
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

#include "netdev/netdev.h"
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
                    FAR const struct sockaddr *addr, socklen_t addrlen);
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
  NULL,            /* si_poll */
  pkt_sendmsg,     /* si_sendmsg */
  pkt_recvmsg,     /* si_recvmsg */
  pkt_close        /* si_close */
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
   * SOCK_RAW and SOCK_CTRL are supported.
   */

  if (psock->s_type == SOCK_RAW || psock->s_type == SOCK_CTRL)
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
  return 0;
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

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              (psock->s_type == SOCK_RAW || psock->s_type == SOCK_CTRL));

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
  int ifindex;

  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_PACKET || addrlen < sizeof(struct sockaddr_ll))
    {
      nerr("ERROR: Invalid address length: %d < %zu\n",
           addrlen, sizeof(struct sockaddr_ll));
      return -EBADF;
    }

  /* Bind a raw socket to a network device. */

  if (psock->s_type == SOCK_RAW || psock->s_type == SOCK_CTRL)
    {
      FAR struct pkt_conn_s *conn = (FAR struct pkt_conn_s *)psock->s_conn;
      FAR struct net_driver_s *dev;

      /* Look at the addr and identify the network interface */

      ifindex = ((FAR struct sockaddr_ll *)addr)->sll_ifindex;

      /* Get the MAC address of that interface */

      dev = netdev_findbyindex(ifindex);
      if (dev == NULL)
        {
          return -EADDRNOTAVAIL;
        }

      /* Only Ethernet is supported */

      if (dev->d_lltype != NET_LL_ETHERNET &&
          dev->d_lltype != NET_LL_IEEE80211)
        {
          return -EAFNOSUPPORT;
        }

      /* Put ifindex and mac address into connection */

      conn->ifindex = ifindex;
      memcpy(conn->lmac, dev->d_mac.ether.ether_addr_octet, 6);

      return OK;
    }
  else
    {
      return -EBADF;
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
      case SOCK_RAW:
      case SOCK_CTRL:
        {
          FAR struct pkt_conn_s *conn = psock->s_conn;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->crefs <= 1)
            {
              /* Yes... free the connection structure */

              conn->crefs = 0;          /* No more references on the connection */
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
