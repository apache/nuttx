/****************************************************************************
 * net/udp/udp_sendto_unbuffered.c
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
#ifdef CONFIG_NET_UDP

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "socket/socket.h"
#include "udp/udp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sendto_s
{
  FAR struct udp_conn_s *st_conn;     /* The UDP connection of interest */
  FAR struct devif_callback_s *st_cb; /* Reference to callback instance */
  FAR struct net_driver_s *st_dev;    /* Driver that will perform the transmission */
  sem_t st_sem;                       /* Semaphore signals sendto completion */
  uint16_t st_buflen;                 /* Length of send buffer (error if <0) */
  const char *st_buffer;              /* Pointer to send buffer */
  int st_sndlen;                      /* Result of the send (length sent or negated errno) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendto_ipselect
 *
 * Description:
 *   If both IPv4 and IPv6 support are enabled, then we will need to select
 *   which one to use when generating the outgoing packet.  If only one
 *   domain is selected, then the setup is already in place and we need do
 *   nothing.
 *
 * Input Parameters:
 *   dev    - The structure of the network driver that caused the event
 *   pstate - sendto state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef NEED_IPDOMAIN_SUPPORT
static inline void sendto_ipselect(FAR struct net_driver_s *dev,
                                   FAR struct sendto_s *pstate)
{
  FAR struct udp_conn_s *conn = pstate->st_conn;
  DEBUGASSERT(conn);

  /* Which domain the socket support */

  if (conn->domain == PF_INET)
    {
      /* Select the IPv4 domain */

      udp_ipv4_select(dev);
    }
  else /* if (psock->s_domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      DEBUGASSERT(conn->domain == PF_INET6);
      udp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: sendto_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev        The structure of the network driver that caused the event
 *   pvpriv     An instance of struct sendto_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static uint16_t sendto_eventhandler(FAR struct net_driver_s *dev,
                                    FAR void *pvpriv, uint16_t flags)
{
  FAR struct sendto_s *pstate = pvpriv;

  DEBUGASSERT(pstate != NULL && pstate->st_dev != NULL);
  if (pstate != NULL)
    {
      /* The TCP socket should be bound to a device.  Make sure that the
       * polling device is the one that we are bound to.
       */

      if (dev != pstate->st_dev)
        {
          /* Ignore this poll.  Wait for the right device */

          return flags;
        }

      ninfo("flags: %04x\n", flags);

      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nwarn("WARNING: Network is down\n");
          pstate->st_sndlen = -ENETUNREACH;
        }

      /* Check if the outgoing packet is available.  It may have been claimed
       * by a sendto event serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & UDP_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * wait for the next polling cycle and check again.
           */

          return flags;
        }

      /* It looks like we are good to send the data */

      else
        {
#ifdef NEED_IPDOMAIN_SUPPORT
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          sendto_ipselect(dev, pstate);
#endif

          /* Copy the user data into d_appdata and send it */

          devif_send(dev, pstate->st_buffer,
                     pstate->st_buflen, udpip_hdrsize(pstate->st_conn));
          if (dev->d_sndlen == 0)
            {
              return flags;
            }

          pstate->st_sndlen = pstate->st_buflen;
        }

      /* Don't allow any further call backs. */

      pstate->st_cb->flags   = 0;
      pstate->st_cb->priv    = NULL;
      pstate->st_cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->st_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_udp_sendto
 *
 * Description:
 *   This function implements the UDP-specific logic of the standard
 *   sendto() socket operation.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 *   NOTE: All input parameters were verified by sendto() before this
 *   function was called.
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See the description in
 *   net/socket/sendto.c for the list of appropriate return value.
 *
 ****************************************************************************/

ssize_t psock_udp_sendto(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags,
                         FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct udp_conn_s *conn;
  struct sendto_s state;
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_type != SOCK_DGRAM ||
      psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* If the UDP socket was previously assigned a remote peer address via
   * connect(), then as with connection-mode socket, sendto() may not be
   * used with a non-NULL destination address.  Normally send() would be
   * used with such connected UDP sockets.
   */

  conn = psock->s_conn;

  if (to != NULL && _SS_ISCONNECTED(conn->sconn.s_flags))
    {
      /* EISCONN - A destination address was specified and the socket is
       * already connected.
       */

      return -EISCONN;
    }

  /* Otherwise, if the socket is not connected, then a destination address
   * must be provided.
   */

  else if (to == NULL && !_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      /* EDESTADDRREQ - The socket is not connection-mode and no peer\
       * address is set.
       */

      return -EDESTADDRREQ;
    }

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
  /* Assure the the IPv4 destination address maps to a valid MAC address in
   * the ARP table.
   */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (psock->s_domain == PF_INET)
#endif
    {
      in_addr_t destipaddr;

      /* Check if the socket is connection mode */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Yes.. use the connected remote address (the 'to' address is
           * null).
           */

          destipaddr = conn->u.ipv4.raddr;
        }
      else
        {
          FAR const struct sockaddr_in *into;

          /* No.. use the destination address provided by the non-NULL 'to'
           * argument.
           */

          into       = (FAR const struct sockaddr_in *)to;
          destipaddr = into->sin_addr.s_addr;
        }

      /* Make sure that the IP address mapping is in the ARP table */

      ret = arp_send(destipaddr);
    }
#endif /* CONFIG_NET_ARP_SEND */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Assure the the IPv6 destination address maps to a valid MAC address in
   * the neighbor table.
   */

#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      FAR const uint16_t *destipaddr;

      /* Check if the socket is connection mode */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Yes.. use the connected remote address (the 'to' address is
           * null).
           */

          destipaddr = conn->u.ipv6.raddr;
        }
      else
        {
          FAR const struct sockaddr_in6 *into;

          /* No.. use the destination address provided by the non-NULL 'to'
           * argument.
           */

          into       = (FAR const struct sockaddr_in6 *)to;
          destipaddr = into->sin6_addr.s6_addr16;
        }

      /* Make sure that the IP address mapping is in the Neighbor Table */

      ret = icmpv6_neighbor(destipaddr);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      nerr("ERROR: Peer not reachable\n");
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct sendto_s));
  nxsem_init(&state.st_sem, 0, 0);

  state.st_buflen = len;
  state.st_buffer = buf;

  /* Save the reference to the conn structure if it will be needed for
   * asynchronous processing.
   */

  state.st_conn   = conn;

  /* Check if the socket is connected */

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      /* No.. Call udp_connect() to set the remote address in the connection
       * structure to the sendto() destination address.
       */

      ret = udp_connect(conn, to);
      if (ret < 0)
        {
          nerr("ERROR: udp_connect failed: %d\n", ret);
          goto errout_with_lock;
        }
    }

  /* Get the device that will handle the remote packet transfers.  This
   * should never be NULL.
   */

  state.st_dev = udp_find_raddr_device(conn, NULL);
  if (state.st_dev == NULL)
    {
      nerr("ERROR: udp_find_raddr_device failed\n");
      ret = -ENETUNREACH;
      goto errout_with_lock;
    }

  /* Make sure that the device is in the UP state */

  if ((state.st_dev->d_flags & IFF_UP) == 0)
    {
      nwarn("WARNING: device is DOWN\n");
      ret = -EHOSTUNREACH;
      goto errout_with_lock;
    }

  /* Set up the callback in the connection */

  ret = -ENOMEM; /* Assume allocation failure */
  state.st_cb = udp_callback_alloc(state.st_dev, conn);
  if (state.st_cb)
    {
      state.st_cb->flags   = (UDP_POLL | NETDEV_DOWN);
      state.st_cb->priv    = (FAR void *)&state;
      state.st_cb->event   = sendto_eventhandler;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(state.st_dev);

      /* Wait for either the receive to complete or for an error/timeout to
       * occur. NOTES:  net_timedwait will also terminate if a signal
       * is received.
       */

      ret = net_timedwait(&state.st_sem,
                          _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      if (ret >= 0)
        {
          /* The result of the sendto operation is the number of bytes
           * transferred.
           */

          ret = state.st_sndlen;
        }

      /* Make sure that no further events are processed */

      udp_callback_free(state.st_dev, conn, state.st_cb);
    }

errout_with_lock:

  /* Release the semaphore */

  nxsem_destroy(&state.st_sem);

  /* Unlock the network and return the result of the sendto() operation */

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: psock_udp_cansend
 *
 * Description:
 *   psock_udp_cansend() returns a value indicating if a write to the socket
 *   would block.  It is still possible that the write may block if another
 *   write occurs first.
 *
 * Input Parameters:
 *   conn     A reference to UDP connection structure.
 *
 * Returned Value:
 *   OK (Always can send).
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

int psock_udp_cansend(FAR struct udp_conn_s *conn)
{
  return OK;
}

#endif /* CONFIG_NET_UDP */
