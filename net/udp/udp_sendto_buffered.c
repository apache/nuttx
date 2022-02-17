/****************************************************************************
 * net/udp/udp_sendto_buffered.c
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

#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP) && \
    defined(CONFIG_NET_UDP_WRITE_BUFFERS)

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_NET_UDP_WRBUFFER_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"
#include "udp/udp.h"
#include "devif/devif.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

#define UDPIPv4BUF ((FAR struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define UDPIPv6BUF ((FAR struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/* Debug */

#ifdef CONFIG_NET_UDP_WRBUFFER_DUMP
#  define BUF_DUMP(msg,buf,len) lib_dumpbuffer(msg,buf,len)
#else
#  define BUF_DUMP(msg,buf,len)
#  undef  UDP_WBDUMP
#  define UDP_WBDUMP(msg,wrb,len,offset)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef NEED_IPDOMAIN_SUPPORT
static inline void sendto_ipselect(FAR struct net_driver_s *dev,
                                   FAR struct udp_conn_s *conn);
#endif
static int sendto_next_transfer(FAR struct udp_conn_s *conn);
static uint16_t sendto_eventhandler(FAR struct net_driver_s *dev,
                                    FAR void *pvconn, FAR void *pvpriv,
                                    uint16_t flags);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_inqueue_wrb_size
 *
 * Description:
 *   Get the in-queued write buffer size from connection
 *
 * Input Parameters:
 *   conn - The UDP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
static uint32_t udp_inqueue_wrb_size(FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;
  FAR sq_entry_t *entry;
  uint32_t total = 0;

  if (conn)
    {
      for (entry = sq_peek(&conn->write_q); entry; entry = sq_next(entry))
        {
          wrb = (FAR struct udp_wrbuffer_s *)entry;
          total += wrb->wb_iob->io_pktlen;
        }
    }

  return total;
}
#endif /* CONFIG_NET_SEND_BUFSIZE */

/****************************************************************************
 * Name: sendto_writebuffer_release
 *
 * Description:
 *   Release the write buffer at the head of the write buffer queue.
 *
 * Input Parameters:
 *   conn  - The UDP connection of interest
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void sendto_writebuffer_release(FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;
  int ret = OK;

  do
    {
      /* Check if the write queue became empty */

      if (sq_empty(&conn->write_q))
        {
          /* Yes.. stifle any further callbacks until more write data is
           * enqueued.
           */

          conn->sndcb->flags = 0;
          conn->sndcb->priv  = NULL;
          conn->sndcb->event = NULL;
          wrb = NULL;

#ifdef CONFIG_NET_UDP_NOTIFIER
          /* Notify any waiters that the write buffers have been drained. */

          udp_writebuffer_signal(conn);
#endif
        }
      else
        {
          /* Remove the write buffer from the head of the write buffer queue
           * and release it.
           */

          wrb = (FAR struct udp_wrbuffer_s *)sq_remfirst(&conn->write_q);
          DEBUGASSERT(wrb != NULL);

          udp_wrbuffer_release(wrb);

          /* Set up for the next packet transfer by setting the connection
           * address to the address of the next packet now at the header of
           * the write buffer queue.
           */

          ret = sendto_next_transfer(conn);
        }
    }
  while (wrb != NULL && ret < 0);

#if CONFIG_NET_SEND_BUFSIZE > 0
  /* Notify the send buffer available if wrbbuffer drained */

  udp_sendbuffer_notify(conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */
}

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
 *   dev   - The structure of the network driver that caused the event
 *   conn  - The UDP connection of interest
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef NEED_IPDOMAIN_SUPPORT
static inline void sendto_ipselect(FAR struct net_driver_s *dev,
                                   FAR struct udp_conn_s *conn)
{
  /* Which domain the socket support */

  if (conn->domain == PF_INET)
    {
      /* Select the IPv4 domain */

      udp_ipv4_select(dev);
    }
  else /* if (conn->domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      DEBUGASSERT(conn->domain == PF_INET6);
      udp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: sendto_next_transfer
 *
 * Description:
 *   Setup for the next packet transfer.  That function is called (1)
 *   psock_udp_sendto() by when the new UDP packet is buffered at the head of
 *   the write queue and (2) by sendto_writebuffer_release() when that
 *   previously queued write buffer was sent and a new write buffer lies at
 *   the head of the write queue.
 *
 * Input Parameters:
 *   conn  - The UDP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sendto_next_transfer(FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;
  FAR struct net_driver_s *dev;

  /* Set the UDP "connection" to the destination address of the write buffer
   * at the head of the queue.
   */

  wrb = (FAR struct udp_wrbuffer_s *)sq_peek(&conn->write_q);
  if (wrb == NULL)
    {
      ninfo("Write buffer queue is empty\n");
      return -ENOENT;
    }

  /* Has this address already been bound to a local port (lport)? */

  if (!conn->lport)
    {
      /* No.. Find an unused local port number and bind it to the
       * connection structure.
       */

      conn->lport = HTONS(udp_select_port(conn->domain, &conn->u));
    }

  /* Get the device that will handle the remote packet transfers.  This
   * should never be NULL.
   *
   * REVISIT:  There is a logical error here for the case where there are
   * multiple network devices.  In that case, the packets may need to be sent
   * in a different order than they were queued.  Forcing FIFO packet
   * transmission could harm performance.
   */

  dev = udp_find_raddr_device(conn, &wrb->wb_dest);
  if (dev == NULL)
    {
      nerr("ERROR: udp_find_raddr_device failed\n");
      return -ENETUNREACH;
    }

  /* Make sure that the device is in the UP state */

  if ((dev->d_flags & IFF_UP) == 0)
    {
      nwarn("WARNING: device is DOWN\n");
      return -EHOSTUNREACH;
    }

  /* If this is not the same device that we used in the last call to
   * udp_callback_alloc(), then we need to release and reallocate the old
   * callback instance.
   */

  if (conn->sndcb != NULL && conn->dev != dev)
    {
      udp_callback_free(conn->dev, conn, conn->sndcb);
      conn->sndcb = NULL;
    }

  /* Allocate resources to receive a callback from this device if the
   * callback is not already in place.
   */

  if (conn->sndcb == NULL)
    {
      conn->sndcb = udp_callback_alloc(dev, conn);
    }

  /* Test if the callback has been allocated */

  if (conn->sndcb == NULL)
    {
      /* A buffer allocation error occurred */

      nerr("ERROR: Failed to allocate callback\n");
      return -ENOMEM;
    }

  conn->dev = dev;

  /* Set up the callback in the connection */

  conn->sndcb->flags = (UDP_POLL | NETDEV_DOWN);
  conn->sndcb->priv  = (FAR void *)conn;
  conn->sndcb->event = sendto_eventhandler;

  /* Notify the device driver of the availability of TX data */

  netdev_txnotify_dev(dev);
  return OK;
}

/****************************************************************************
 * Name: sendto_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t sendto_eventhandler(FAR struct net_driver_s *dev,
                                    FAR void *pvconn, FAR void *pvpriv,
                                    uint16_t flags)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)pvpriv;

  DEBUGASSERT(dev != NULL && conn != NULL);

  ninfo("flags: %04x\n", flags);

  /* Check if the network device has gone down  */

  if ((flags & NETDEV_DOWN) != 0)
    {
      ninfo("Device down: %04x\n", flags);

      /* Free the write buffer at the head of the queue and attempt to setup
       * the next transfer.
       */

      sendto_writebuffer_release(conn);
      return flags;
    }

  /* The UDP socket should be bound to a device.  Make sure that the polling
   * device is the one that we are bound to.
   *
   * REVISIT:  There is a logical error here for the case where there are
   * multiple network devices.  In that case, the packets may need to be sent
   * in a different order than they were queued.  The packet we may need to
   * send on this device may not be at the head of the list.  Forcing FIFO
   * packet transmission could degrade performance!
   */

  DEBUGASSERT(conn != NULL);
  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      return flags;
    }

  /* Check for a normal polling cycle and if the outgoing packet is
   * available.  It would not be available if it has been claimed by a send
   * event serving a different thread -OR- if the output buffer currently
   * contains unprocessed incoming data.  In these cases we will just have
   * to wait for the next polling cycle.
   *
   * And, of course, we can do nothing if we have no data in the write
   * buffers to send.
   */

  if (dev->d_sndlen <= 0 && (flags & UDP_NEWDATA) == 0 &&
      (flags & UDP_POLL) != 0 && !sq_empty(&conn->write_q))
    {
      FAR struct udp_wrbuffer_s *wrb;
      size_t sndlen;

      /* Peek at the head of the write queue (but don't remove anything
       * from the write queue yet).  We know from the above test that
       * the write_q is not empty.
       */

      wrb = (FAR struct udp_wrbuffer_s *)sq_peek(&conn->write_q);
      DEBUGASSERT(wrb != NULL);

      /* If the udp socket not connected, it is possible to have
       * multi-different destination address in each iob entry,
       * update the remote address every time to avoid sent to the
       * incorrect destination.
       */

      udp_connect(conn, (FAR const struct sockaddr *)&wrb->wb_dest);

      /* Get the amount of data that we can send in the next packet.
       * We will send either the remaining data in the buffer I/O
       * buffer chain, or as much as will fit given the MSS and current
       * window size.
       */

      sndlen = wrb->wb_iob->io_pktlen;
      ninfo("wrb=%p sndlen=%zu\n", wrb, sndlen);

#ifdef NEED_IPDOMAIN_SUPPORT
      /* If both IPv4 and IPv6 support are enabled, then we will need to
       * select which one to use when generating the outgoing packet.
       * If only one domain is selected, then the setup is already in
       * place and we need do nothing.
       */

      sendto_ipselect(dev, conn);
#endif
      /* Then set-up to send that amount of data with the offset
       * corresponding to the size of the IP-dependent address structure.
       */

      devif_iob_send(dev, wrb->wb_iob, sndlen, 0);

      /* Free the write buffer at the head of the queue and attempt to
       * setup the next transfer.
       */

      sendto_writebuffer_release(conn);

      /* Only one data can be sent by low level driver at once,
       * tell the caller stop polling the other connections.
       */

      flags &= ~UDP_POLL;
    }

  /* Continue waiting */

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
  FAR struct udp_wrbuffer_s *wrb;
  FAR struct udp_conn_s *conn;
  bool nonblock;
  bool empty;
  int ret = OK;

  /* Get the underlying the UDP connection structure.  */

  conn = psock->s_conn;
  DEBUGASSERT(conn);

  /* If the UDP socket was previously assigned a remote peer address via
   * connect(), then as with connection-mode socket, sendto() may not be
   * used with a non-NULL destination address.  Normally send() would be
   * used with such connected UDP sockets.
   */

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
      /* EDESTADDRREQ - The socket is not connection-mode and no peer
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
      nerr("ERROR: Not reachable\n");
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
                            (flags & MSG_DONTWAIT) != 0;

  /* Dump the incoming buffer */

  BUF_DUMP("psock_udp_sendto", buf, len);

  if (len > 0)
    {
      net_lock();

#if CONFIG_NET_SEND_BUFSIZE > 0
      /* If the send buffer size exceeds the send limit,
       * wait for the write buffer to be released
       */

      while (udp_inqueue_wrb_size(conn) + len > conn->sndbufs)
        {
          if (nonblock)
            {
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          net_lockedwait_uninterruptible(&conn->sndsem);
        }
#endif /* CONFIG_NET_SEND_BUFSIZE */

      /* Allocate a write buffer.  Careful, the network will be momentarily
       * unlocked here.
       */

      if (nonblock)
        {
          wrb = udp_wrbuffer_tryalloc();
        }
      else
        {
          wrb = udp_wrbuffer_alloc();
        }

      if (wrb == NULL)
        {
          /* A buffer allocation error occurred */

          nerr("ERROR: Failed to allocate write buffer\n");
          ret = nonblock ? -EAGAIN : -ENOMEM;
          goto errout_with_lock;
        }

      /* Initialize the write buffer
       *
       * Check if the socket is connected
       */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Yes.. get the connection address from the connection structure */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (conn->domain == PF_INET)
#endif
            {
              FAR struct sockaddr_in *addr4 =
                (FAR struct sockaddr_in *)&wrb->wb_dest;

              addr4->sin_family = AF_INET;
              addr4->sin_port   = conn->rport;
              net_ipv4addr_copy(addr4->sin_addr.s_addr, conn->u.ipv4.raddr);
              memset(addr4->sin_zero, 0, sizeof(addr4->sin_zero));
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
              FAR struct sockaddr_in6 *addr6 =
                (FAR struct sockaddr_in6 *)&wrb->wb_dest;

              addr6->sin6_family = AF_INET6;
              addr6->sin6_port   = conn->rport;
              net_ipv6addr_copy(addr6->sin6_addr.s6_addr,
                                conn->u.ipv6.raddr);
            }
#endif /* CONFIG_NET_IPv6 */
        }

      /* Not connected.  Use the provided destination address */

      else
        {
          memcpy(&wrb->wb_dest, to, tolen);
        }

      /* Copy the user data into the write buffer.  We cannot wait for
       * buffer space if the socket was opened non-blocking.
       */

      if (nonblock)
        {
          ret = iob_trycopyin(wrb->wb_iob, (FAR uint8_t *)buf, len, 0, false,
                              IOBUSER_NET_SOCK_UDP);
        }
      else
        {
          unsigned int count;
          int blresult;

          /* iob_copyin might wait for buffers to be freed, but if
           * network is locked this might never happen, since network
           * driver is also locked, therefore we need to break the lock
           */

          blresult = net_breaklock(&count);
          ret = iob_copyin(wrb->wb_iob, (FAR uint8_t *)buf, len, 0, false,
                           IOBUSER_NET_SOCK_UDP);
          if (blresult >= 0)
            {
              net_restorelock(count);
            }
        }

      if (ret < 0)
        {
          goto errout_with_wrb;
        }

      /* Dump I/O buffer chain */

      UDP_WBDUMP("I/O buffer chain", wrb, wrb->wb_iob->io_pktlen, 0);

      /* sendto_eventhandler() will send data in FIFO order from the
       * conn->write_q.
       *
       * REVISIT:  Why FIFO order?  Because it is easy.  In a real world
       * environment where there are multiple network devices this might
       * be inefficient because we could be sending data to different
       * device out-of-queued-order to optimize performance.  Sending
       * data to different networks from a single UDP socket is probably
       * not a very common use case, however.
       */

      empty = sq_empty(&conn->write_q);

      sq_addlast(&wrb->wb_node, &conn->write_q);
      ninfo("Queued WRB=%p pktlen=%u write_q(%p,%p)\n",
            wrb, wrb->wb_iob->io_pktlen,
            conn->write_q.head, conn->write_q.tail);

      if (empty)
        {
          /* The new write buffer lies at the head of the write queue.  Set
           * up for the next packet transfer by setting the connection
           * address to the address of the next packet now at the header of
           * the write buffer queue.
           */

          ret = sendto_next_transfer(conn);
          if (ret < 0)
            {
              sq_remlast(&conn->write_q);
              goto errout_with_wrb;
            }
        }

      net_unlock();
    }

  /* Return the number of bytes that will be sent */

  return len;

errout_with_wrb:
  udp_wrbuffer_release(wrb);

errout_with_lock:
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: psock_udp_cansend
 *
 * Description:
 *   psock_udp_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   conn     A reference to UDP connection structure.
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *
 ****************************************************************************/

int psock_udp_cansend(FAR struct udp_conn_s *conn)
{
  /* Verify that we received a valid socket */

  if (conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* In order to setup the send, we need to have at least one free write
   * buffer head and at least one free IOB to initialize the write buffer
   * head.
   *
   * REVISIT:  The send will still block if we are unable to buffer the
   * entire user-provided buffer which may be quite large.  We will almost
   * certainly need to have more than one free IOB, but we don't know how
   * many more.
   */

  if (udp_wrbuffer_test() < 0 || iob_navail(false) <= 0)
    {
      return -EWOULDBLOCK;
    }

  return OK;
}

/****************************************************************************
 * Name: udp_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The UDP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
void udp_sendbuffer_notify(FAR struct udp_conn_s *conn)
{
  int val = 0;

  nxsem_get_value(&conn->sndsem, &val);
  if (val < 0)
    {
      nxsem_post(&conn->sndsem);
    }
}
#endif /* CONFIG_NET_SEND_BUFSIZE */

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NET_UDP_WRITE_BUFFERS */
