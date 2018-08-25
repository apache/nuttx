/****************************************************************************
 * net/udp/udp_send_buffered.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/clock.h>
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

#define UDPIPv4BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define UDPIPv6BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

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
#ifdef CONFIG_NET_ETHERNET
static inline bool sendto_addrcheck(FAR struct udp_conn_s *conn,
                                    FAR struct net_driver_s *dev);
#else
#  define sendto_addrcheck(c,d) (true)
#endif
#ifdef CONFIG_NET_SOCKOPTS
static inline int sendto_timeout(FAR struct socket *psock,
                                 FAR struct udp_conn_s *conn);
#endif
static int sendto_next_transfer(FAR struct socket *psock,
                                FAR struct udp_conn_s *conn);
static uint16_t sendto_eventhandler(FAR struct net_driver_s *dev,
                                    FAR void *pvconn, FAR void *pvpriv,
                                    uint16_t flags);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendto_writebuffer_release
 *
 * Description:
 *   Release the write buffer at the head of the write buffer queue.
 *
 * Input Parameters:
 *   dev   - The structure of the network driver that caused the event
 *   psock - Socket state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void sendto_writebuffer_release(FAR struct socket *psock,
                                       FAR struct udp_conn_s *conn)
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

          psock->s_sndcb->flags = 0;
          psock->s_sndcb->priv  = NULL;
          psock->s_sndcb->event = NULL;
          wrb = NULL;
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

         ret = sendto_next_transfer(psock, conn);
       }
    }
  while (wrb != NULL && ret < 0);
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
 *   psock - Socket state structure
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
 * Name: sendto_addrcheck
 *
 * Description:
 *   Check if the destination IP address is in the IPv4 ARP or IPv6 Neighbor
 *   tables.  If not, then the send won't actually make it out... it will be
 *   replaced with an ARP request (IPv4) or a Neighbor Solicitation (IPv6).
 *
 *   NOTE 1: This could be an expensive check if there are a lot of
 *   entries in the ARP or Neighbor tables.
 *
 *   NOTE 2: If we are actually harvesting IP addresses on incoming IP
 *   packets, then this check should not be necessary; the MAC mapping
 *   should already be in the ARP table in many cases (IPv4 only).
 *
 *   NOTE 3: If CONFIG_NET_ARP_SEND then we can be assured that the IP
 *   address mapping is already in the ARP table.
 *
 * Input Parameters:
 *   conn - The UDP connection structure
 *   dev  - Polling network device
 *
 * Returned Value:
 *   true - The Ethernet MAC address is in the ARP or Neighbor table (OR
 *          the network device is not Ethernet).
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static inline bool sendto_addrcheck(FAR struct udp_conn_s *conn,
                                    FAR struct net_driver_s *dev)
{
  /* REVISIT: Could the MAC address not also be in a routing table? */

  if (dev->d_lltype != NET_LL_ETHERNET)
    {
      return true;
    }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
#if !defined(CONFIG_NET_ARP_IPIN) && !defined(CONFIG_NET_ARP_SEND)
      return (arp_find(conn->u.ipv4.raddr, NULL) >= 0);
#else
      return true;
#endif
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
#if !defined(CONFIG_NET_ICMPv6_NEIGHBOR)
      return (neighbor_findentry(conn->u.ipv6.raddr) != NULL);
#else
      return true;
#endif
    }
#endif /* CONFIG_NET_IPv6 */
}
#endif /* CONFIG_NET_ETHERNET */

/****************************************************************************
 * Name: sendto_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Input Parameters:
 *   pstate - sendto state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static inline int sendto_timeout(FAR struct socket *psock,
                                 FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;

  /* Peek at the head of the write queue (without altering the write queue). */

  wrb = (FAR struct udp_wrbuffer_s *)sq_peek(&conn->write_q);
  if (wrb != NULL)
    {
      /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
       * If none... we well let the send wait forever.
       */

      if (psock->s_sndtimeo != 0)
        {
          /* Check if the configured timeout has elapsed */

          return net_timeo(wrb->wb_start, psock->s_sndtimeo);
        }
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS */

/****************************************************************************
 * Name: sendto_next_transfer
 *
 * Description:
 *   Setup for the next packet transfer
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The UDP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sendto_next_transfer(FAR struct socket *psock,
                                FAR struct udp_conn_s *conn)
{
  FAR struct udp_wrbuffer_s *wrb;
  FAR struct net_driver_s *dev;
  int ret;

  /* Set the UDP "connection" to the destination address of the write buffer
   * at the head of the queue.
   */

  wrb = (FAR struct udp_wrbuffer_s *)sq_peek(&conn->write_q);
  if (wrb == NULL)
    {
      ninfo("Write buffer queue is empty\n");
      return -ENOENT;
    }

  ret = udp_connect(conn, (FAR const struct sockaddr *)&wrb->wb_dest);
  if (ret < 0)
    {
      nerr("ERROR: udp_connect failed: %d\n", ret);
      return ret;
    }

 /* Get the device that will handle the remote packet transfers.  This
  * should never be NULL.
  */

 dev = udp_find_raddr_device(conn);
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

  if (psock->s_sndcb != NULL && conn->dev != dev)
    {
      udp_callback_free(conn->dev, conn, psock->s_sndcb);
      psock->s_sndcb = NULL;
    }

  /* Allocate resources to receive a callback from this device if the
   * callback is not already in place.
   */

  if (psock->s_sndcb == NULL)
    {
      psock->s_sndcb = udp_callback_alloc(dev, conn);
    }

  /* Test if the callback has been allocated */

  if (psock->s_sndcb == NULL)
    {
      /* A buffer allocation error occurred */

      nerr("ERROR: Failed to allocate callback\n");
      return -ENOMEM;
    }

  conn->dev = dev;

  /* Set up the callback in the connection */

  psock->s_sndcb->flags = (UDP_POLL | NETDEV_DOWN);
  psock->s_sndcb->priv  = (FAR void *)psock;
  psock->s_sndcb->event = sendto_eventhandler;

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
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)pvconn;
  FAR struct socket *psock = (FAR struct socket *)pvpriv;

  ninfo("flags: %04x\n", flags);

  /* Check if the network device has gone down  */

  if ((flags & NETDEV_DOWN) != 0)
    {
      ninfo("Device down: %04x\n", flags);

      /* Free the write buffer at the head of the queue and attempt to setup
       * the next transfer.
       */

      sendto_writebuffer_release(psock, conn);
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
      /* Check if the destination IP address is in the ARP  or Neighbor
       * table.  If not, then the send won't actually make it out... it
       * will be replaced with an ARP request or Neighbor Solicitation.
       */

      if (sendto_addrcheck(conn, dev))
        {
          FAR struct udp_wrbuffer_s *wrb;
          size_t sndlen;

          /* Peek at the head of the write queue (but don't remove anything
           * from the write queue yet).  We know from the above test that
           * the write_q is not empty.
           */

          wrb = (FAR struct udp_wrbuffer_s *)sq_peek(&conn->write_q);
          DEBUGASSERT(wrb != NULL);

          /* Get the amount of data that we can send in the next packet.
           * We will send either the remaining data in the buffer I/O
           * buffer chain, or as much as will fit given the MSS and current
           * window size.
           */

          sndlen = wrb->wb_iob->io_pktlen;
          ninfo("wrb=%p sndlen=%u\n", wrb, sndlen);

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

          sendto_writebuffer_release(psock, conn);

          /* Only one data can be sent by low level driver at once,
           * tell the caller stop polling the other connections.
           */

          flags &= ~UDP_POLL;
        }
    }

#ifdef CONFIG_NET_SOCKOPTS
  /* We cannot send the data now.  Check for a timeout. */

  else if (sendto_timeout(psock, conn))
    {
      /* Free the write buffer at the head of the queue and attempt to setup
       * the next transfer.
       */

      sendto_writebuffer_release(psock, conn);
    }
#endif /* CONFIG_NET_SOCKOPTS */

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
                         size_t len, int flags, FAR const struct sockaddr *to,
                         socklen_t tolen)
{
  FAR struct udp_conn_s *conn;
  FAR struct udp_wrbuffer_s *wrb;
  int ret = OK;

  /* If the UDP socket was previously assigned a remote peer address via
   * connect(), then as with connection-mode socket, sendto() may not be
   * used with a non-NULL destination address.  Normally send() would be
   * used with such connected UDP sockets.
   */

  if (to != NULL && _SS_ISCONNECTED(psock->s_flags))
    {
      /* EISCONN - A destination address was specified and the socket is
       * already connected.
       */

      return -EISCONN;
    }

  /* Otherwise, if the socket is not connected, then a destination address
   * must be provided.
   */

  else if (to == NULL && !_SS_ISCONNECTED(psock->s_flags))
    {
      /* EDESTADDRREQ - The socket is not connection-mode and no peer
       * address is set.
       */

      return -EDESTADDRREQ;
    }

  /* Get the underlying the UDP connection structure.  */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn);

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

      if (_SS_ISCONNECTED(psock->s_flags))
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

      if (_SS_ISCONNECTED(psock->s_flags))
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

  /* Dump the incoming buffer */

  BUF_DUMP("psock_udp_send", buf, len);

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  if (len > 0)
    {
      /* Allocate a write buffer.  Careful, the network will be momentarily
       * unlocked here.
       */

      net_lock();
      wrb = udp_wrbuffer_alloc();
      if (wrb == NULL)
        {
          /* A buffer allocation error occurred */

          nerr("ERROR: Failed to allocate write buffer\n");
          ret = -ENOMEM;
          goto errout_with_lock;
        }

      /* Initialize the write buffer */
      /* Check if the socket is connected */

      if (_SS_ISCONNECTED(psock->s_flags))
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
              net_ipv6addr_copy(addr6->sin6_addr.s6_addr, conn->u.ipv6.raddr);
            }
#endif /* CONFIG_NET_IPv6 */
        }

      /* Not connected.  Use the provided destination address */

      else
        {
          memcpy(&wrb->wb_dest, to, tolen);
        }

#ifdef CONFIG_NET_SOCKOPTS
      wrb->wb_start = clock_systimer();
#endif

      /* Copy the user data into the write buffer.  We cannot wait for
       * buffer space if the socket was opened non-blocking.
       */

      if (_SS_ISNONBLOCK(psock->s_flags))
        {
          ret = iob_trycopyin(wrb->wb_iob, (FAR uint8_t *)buf, len, 0, false);
        }
      else
        {
          ret = iob_copyin(wrb->wb_iob, (FAR uint8_t *)buf, len, 0, false);
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

      sq_addlast(&wrb->wb_node, &conn->write_q);
      ninfo("Queued WRB=%p pktlen=%u write_q(%p,%p)\n",
            wrb, wrb->wb_iob->io_pktlen,
            conn->write_q.head, conn->write_q.tail);

      /* Set up for the next packet transfer by setting the connection
       * address to the address of the next packet now at the header of the
       * write buffer queue.
       */

     ret = sendto_next_transfer(psock, conn);
     if (ret < 0)
       {
         (void)sq_remlast(&conn->write_q);
         goto errout_with_wrb;
       }

      net_unlock();
    }

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Return the number of bytes that will be sent */

  return len;

errout_with_wrb:
  udp_wrbuffer_release(wrb);

errout_with_lock:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP && CONFIG_NET_UDP_WRITE_BUFFERS */
