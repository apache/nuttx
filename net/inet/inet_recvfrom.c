/****************************************************************************
 * net/inet/inet_recvfrom.c
 *
 *   Copyright (C) 2007-2009, 2011-2018 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"
#include "socket/socket.h"
#include "usrsock/usrsock.h"
#include "inet/inet.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF    ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF    ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define UDPIPv4BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define UDPIPv6BUF ((struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

#define TCPIPv4BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
struct inet_recvfrom_s
{
  FAR struct socket       *ir_sock;      /* The parent socket structure */
#ifdef CONFIG_NET_SOCKOPTS
  clock_t                  ir_starttime; /* rcv start time for determining timeout */
#endif
  FAR struct devif_callback_s *ir_cb;    /* Reference to callback instance */
  sem_t                    ir_sem;       /* Semaphore signals recv completion */
  size_t                   ir_buflen;    /* Length of receive buffer */
  uint8_t                 *ir_buffer;    /* Pointer to receive buffer */
  FAR struct sockaddr     *ir_from;      /* Address of sender */
  FAR socklen_t           *ir_fromlen;   /* Number of bytes allocated for address of sender */
  ssize_t                  ir_recvlen;   /* The received length */
  int                      ir_result;    /* Success:OK, failure:negated errno */
};
#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_update_recvlen
 *
 * Description:
 *   Update information about space available for new data and update size
 *   of data in buffer,  This logic accounts for the case where
 *   inet_udp_readahead() sets state.ir_recvlen == -1 .
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *   recvlen  size of new data appended to buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)

static inline void inet_update_recvlen(FAR struct inet_recvfrom_s *pstate,
                                       size_t recvlen)
{
  if (pstate->ir_recvlen < 0)
    {
      pstate->ir_recvlen = 0;
    }

  pstate->ir_recvlen += recvlen;
  pstate->ir_buffer  += recvlen;
  pstate->ir_buflen  -= recvlen;
}
#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   The number of bytes taken from the packet.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
static size_t inet_recvfrom_newdata(FAR struct net_driver_s *dev,
                                    FAR struct inet_recvfrom_s *pstate)
{
  size_t recvlen;

  /* Get the length of the data to return */

  if (dev->d_len > pstate->ir_buflen)
    {
      recvlen = pstate->ir_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new appdata into the user buffer */

  memcpy(pstate->ir_buffer, dev->d_appdata, recvlen);
  ninfo("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  inet_update_recvlen(pstate, recvlen);

  return recvlen;
}
#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_tcp_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static inline void inet_tcp_newdata(FAR struct net_driver_s *dev,
                                    FAR struct inet_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  size_t recvlen = inet_recvfrom_newdata(dev, pstate);

  /* If there is more data left in the packet that we could not buffer, then
   * add it to the read-ahead buffers.
   */

  if (recvlen < dev->d_len)
    {
#ifdef CONFIG_NET_TCP_READAHEAD
      FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pstate->ir_sock->s_conn;
      FAR uint8_t *buffer = (FAR uint8_t *)dev->d_appdata + recvlen;
      uint16_t buflen = dev->d_len - recvlen;
#ifdef CONFIG_DEBUG_NET
      uint16_t nsaved;

      nsaved = tcp_datahandler(conn, buffer, buflen);
#else
      (void)tcp_datahandler(conn, buffer, buflen);
#endif

      /* There are complicated buffering issues that are not addressed fully
       * here.  For example, what if up_datahandler() cannot buffer the
       * remainder of the packet?  In that case, the data will be dropped but
       * still ACKed.  Therefore it would not be resent.
       *
       * This is probably not an issue here because we only get here if the
       * read-ahead buffers are empty and there would have to be something
       * serioulsy wrong with the configuration not to be able to buffer a
       * partial packet in this context.
       */

#ifdef CONFIG_DEBUG_NET
      if (nsaved < buflen)
        {
          nerr("ERROR: packet data not saved (%d bytes)\n", buflen - nsaved);
        }
#endif
#else
      nerr("ERROR: packet data lost (%d bytes)\n", dev->d_len - recvlen);
#endif
   }

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The sructure of the network driver that generated the event
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static inline void inet_udp_newdata(FAR struct net_driver_s *dev,
                                    FAR struct inet_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  (void)inet_recvfrom_newdata(dev, pstate);

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_tcp_readahead and inet_udp_readahead
 *
 * Description:
 *   Copy the read-ahead data from the packet
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(NET_TCP_HAVE_STACK) && defined(CONFIG_NET_TCP_READAHEAD)
static inline void inet_tcp_readahead(struct inet_recvfrom_s *pstate)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pstate->ir_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any TCP data already buffered in a read-ahead
   * buffer.
   */

  while ((iob = iob_peek_queue(&conn->readahead)) != NULL &&
          pstate->ir_buflen > 0)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(pstate->ir_buffer, iob, pstate->ir_buflen, 0);
      ninfo("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

      /* Update the accumulated size of the data read */

      inet_update_recvlen(pstate, recvlen);

      /* If we took all of the ata from the I/O buffer chain is empty, then
       * release it.  If there is still data available in the I/O buffer
       * chain, then just trim the data that we have taken from the
       * beginning of the I/O buffer chain.
       */

      if (recvlen >= iob->io_pktlen)
        {
          FAR struct iob_s *tmp;

          /* Remove the I/O buffer chain from the head of the read-ahead
           * buffer queue.
           */

          tmp = iob_remove_queue(&conn->readahead);
          DEBUGASSERT(tmp == iob);
          UNUSED(tmp);

          /* And free the I/O buffer chain */

          (void)iob_free_chain(iob);
        }
      else
        {
          /* The bytes that we have received from the head of the I/O
           * buffer chain (probably changing the head of the I/O
           * buffer queue).
           */

          (void)iob_trimhead_queue(&conn->readahead, recvlen);
        }
    }
}
#endif /* NET_TCP_HAVE_STACK && CONFIG_NET_TCP_READAHEAD */

#if defined(NET_UDP_HAVE_STACK) && defined(CONFIG_NET_UDP_READAHEAD)

static inline void inet_udp_readahead(struct inet_recvfrom_s *pstate)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)pstate->ir_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any UDP datagram already buffered in a read-ahead
   * buffer.
   */

  pstate->ir_recvlen = -1;

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      FAR struct iob_s *tmp;
      uint8_t src_addr_size;

      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(&src_addr_size, iob, sizeof(uint8_t), 0);
      if (recvlen != sizeof(uint8_t))
        {
          goto out;
        }

      if (0
#ifdef CONFIG_NET_IPv6
          || src_addr_size == sizeof(struct sockaddr_in6)
#endif
#ifdef CONFIG_NET_IPv4
          || src_addr_size == sizeof(struct sockaddr_in)
#endif
        )
        {
          if (pstate->ir_from)
            {
              socklen_t len = *pstate->ir_fromlen;
              len = (socklen_t)src_addr_size > len ? len : (socklen_t)src_addr_size;

              recvlen = iob_copyout((FAR uint8_t *)pstate->ir_from, iob,
                                    len, sizeof(uint8_t));
              if (recvlen != len)
                {
                  goto out;
                }
            }
        }

      if (pstate->ir_buflen > 0)
        {
          recvlen = iob_copyout(pstate->ir_buffer, iob, pstate->ir_buflen,
                                src_addr_size + sizeof(uint8_t));

          ninfo("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

          /* Update the accumulated size of the data read */

          pstate->ir_recvlen  = recvlen;
          pstate->ir_buffer  += recvlen;
          pstate->ir_buflen  -= recvlen;
        }
      else
        {
          pstate->ir_recvlen = 0;
        }

out:
      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      tmp = iob_remove_queue(&conn->readahead);
      DEBUGASSERT(tmp == iob);
      UNUSED(tmp);

      /* And free the I/O buffer chain */

      (void)iob_free_chain(iob);
    }
}
#endif

/****************************************************************************
 * Name: inet_recvfrom_timeout
 *
 * Description:
 *   Check for recvfrom timeout.
 *
 * Input Parameters:
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
#ifdef CONFIG_NET_SOCKOPTS
static int inet_recvfrom_timeout(struct inet_recvfrom_s *pstate)
{
  FAR struct socket *psock = 0;
  socktimeo_t        timeo = 0;

  /* Check for a timeout configured via setsockopts(SO_RCVTIMEO). If none...
   * we well let the read hang forever (except for the special case below).
   */

  /* Get the socket reference from the private data */

  psock = pstate->ir_sock;
  if (psock)
    {
      /* Recover the timeout value (zero if no timeout) */

      timeo = psock->s_rcvtimeo;
    }

  /* Use a fixed, configurable delay under the following circumstances:
   *
   * 1) This delay function has been enabled with CONFIG_NET_TCP_RECVDELAY > 0
   * 2) Some data has already been received from the socket.  Since this can
   *    only be true for a TCP/IP socket, this logic applies only to TCP/IP
   *    sockets.  And either
   * 3) There is no configured receive timeout, or
   * 4) The configured receive timeout is greater than than the delay
   */

#if CONFIG_NET_TCP_RECVDELAY > 0
  if ((timeo == 0 || timeo > CONFIG_NET_TCP_RECVDELAY) &&
      pstate->ir_recvlen > 0)
    {
      /* Use the configured timeout */

      timeo = CONFIG_NET_TCP_RECVDELAY;
    }
#endif

  /* Is there an effective timeout? */

  if (timeo)
    {
      /* Yes.. Check if the timeout has elapsed */

      return net_timeo(pstate->ir_starttime, timeo);
    }

  /* No timeout -- hang forever waiting for data. */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS */
#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_tcp_sender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Input Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static inline void inet_tcp_sender(FAR struct net_driver_s *dev,
                                   FAR struct inet_recvfrom_s *pstate)
{
  /* Get the family from the packet type, IP address from the IP header, and
   * the port number from the TCP header.
   */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct sockaddr_in6 *infrom =
        (FAR struct sockaddr_in6 *)pstate->ir_from;

      if (infrom)
        {
          FAR struct tcp_hdr_s *tcp   = TCPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

          infrom->sin6_family = AF_INET6;
          infrom->sin6_port   = tcp->srcport;

          net_ipv6addr_copy(infrom->sin6_addr.s6_addr, ipv6->srcipaddr);
        }
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct sockaddr_in *infrom  =
        (FAR struct sockaddr_in *)pstate->ir_from;

      if (infrom)
        {
          FAR struct tcp_hdr_s *tcp   = TCPIPv4BUF;
          FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

          infrom->sin_family = AF_INET;
          infrom->sin_port   = tcp->srcport;

          net_ipv4addr_copy(infrom->sin_addr.s_addr,
                            net_ip4addr_conv32(ipv4->srcipaddr));
        }
    }
#endif /* CONFIG_NET_IPv4 */
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_tcp_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   TCP receive operation via by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static uint16_t inet_tcp_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvconn, FAR void *pvpriv,
                                      uint16_t flags)
{
  FAR struct inet_recvfrom_s *pstate = (struct inet_recvfrom_s *)pvpriv;

#if 0 /* REVISIT: The assertion fires.  Why? */
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the own that we are bound to.
   */

  DEBUGASSERT(conn->dev == NULL || conn->dev == dev);
  if (conn->dev != NULL && conn->dev != dev)
    {
      return flags;
    }
#endif

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If new data is available, then complete the read action. */

      if ((flags & TCP_NEWDATA) != 0)
        {
          /* Copy the data from the packet (saving any unused bytes from the
           * packet in the read-ahead buffer).
           */

          inet_tcp_newdata(dev, pstate);

          /* Save the sender's address in the caller's 'from' location */

          inet_tcp_sender(dev, pstate);

          /* Indicate that the data has been consumed and that an ACK
           * should be sent.
           */

          flags = (flags & ~TCP_NEWDATA) | TCP_SNDACK;

          /* Check for transfer complete.  We will consider the transfer
           * complete in own of two different ways, depending on the setting
           * of CONFIG_NET_TCP_RECVDELAY.
           *
           * 1) If CONFIG_NET_TCP_RECVDELAY == 0 then we will consider the
           *    TCP/IP transfer complete as soon as any data has been received.
           *    This is safe because if any additional data is received, it
           *    will be retained in the TCP/IP read-ahead buffer until the
           *    next receive is performed.
           * 2) CONFIG_NET_TCP_RECVDELAY > 0 may be set to wait a little
           *    bit to determine if more data will be received.  You might
           *    do this if read-ahead buffering is disabled and we want to
           *    minimize the loss of back-to-back packets.  In this case,
           *    the transfer is complete when either a) the entire user buffer
           *    is full or 2) when the receive timeout occurs (below).
           */

#if CONFIG_NET_TCP_RECVDELAY > 0
          if (pstate->ir_buflen == 0)
#else
          if (pstate->ir_recvlen > 0)
#endif
            {
              ninfo("TCP resume\n");

              /* The TCP receive buffer is non-empty.  Return now and don't
               * allow any further TCP call backs.
               */

              pstate->ir_cb->flags   = 0;
              pstate->ir_cb->priv    = NULL;
              pstate->ir_cb->event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              nxsem_post(&pstate->ir_sem);
            }

#ifdef CONFIG_NET_SOCKOPTS
          /* Reset the timeout.  We will want a short timeout to terminate
           * the TCP receive.
           */

          pstate->ir_starttime = clock_systimer();
#endif
        }

      /* Check for a loss of connection.
       *
       * TCP_DISCONN_EVENTS:
       *   TCP_CLOSE:    The remote host has closed the connection
       *   TCP_ABORT:    The remote host has aborted the connection
       *   TCP_TIMEDOUT: Connection aborted due to too many retransmissions.
       *   NETDEV_DOWN:  The network device went down
       */

      else if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          FAR struct socket *psock = pstate->ir_sock;

          nwarn("WARNING: Lost connection\n");

          /* We could get here recursively through the callback actions of
           * tcp_lost_connection().  So don't repeat that action if we have
           * already been disconnected.
           */

          DEBUGASSERT(psock != NULL);
          if (_SS_ISCONNECTED(psock->s_flags))
            {
              /* Handle loss-of-connection event */

              tcp_lost_connection(psock, pstate->ir_cb, flags);
            }

          /* Check if the peer gracefully closed the connection. */

          if ((flags & TCP_CLOSE) != 0)
            {
              /* This case should always return success (zero)! The value of
               * ir_recvlen, if zero, will indicate that the connection was
               * gracefully closed.
               */

              pstate->ir_result = 0;
            }
          else
            {
              /* If no data has been received, then return ENOTCONN.
               * Otherwise, let this return success.  The failure will
               * be reported the next time that recv[from]() is called.
               */

#if CONFIG_NET_TCP_RECVDELAY > 0
              if (pstate->ir_recvlen > 0)
                {
                  pstate->ir_result = 0;
                }
              else
                {
                  pstate->ir_result = -ENOTCONN;
                }
#else
              pstate->ir_result = -ENOTCONN;
#endif
            }

          /* Wake up the waiting thread */

          nxsem_post(&pstate->ir_sem);
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

      else if (inet_recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          ninfo("TCP timeout\n");

          pstate->ir_cb->flags   = 0;
          pstate->ir_cb->priv    = NULL;
          pstate->ir_cb->event   = NULL;

          /* Report an error only if no data has been received. (If
           * CONFIG_NET_TCP_RECVDELAY then ir_recvlen should always be
           * less than or equal to zero).
           */

#if CONFIG_NET_TCP_RECVDELAY > 0
          if (pstate->ir_recvlen <= 0)
#endif
            {
              /* Report the timeout error */

              pstate->ir_result = -EAGAIN;
            }

          /* Wake up the waiting thread, returning either the error -EAGAIN
           * that signals the timeout event or the data received up to
           * the point that the timeout occurred (no error).
           */

          nxsem_post(&pstate->ir_sem);
        }
#endif /* CONFIG_NET_SOCKOPTS */
    }

  return flags;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_sender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Input Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static inline void inet_udp_sender(struct net_driver_s *dev, struct inet_recvfrom_s *pstate)
{
  /* Get the family from the packet type, IP address from the IP header, and
   * the port number from the UDP header.
   */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct sockaddr_in6 *infrom =
        (FAR struct sockaddr_in6 *)pstate->ir_from;
      FAR socklen_t *fromlen = pstate->ir_fromlen;

      if (infrom)
        {
          FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

          infrom->sin6_family = AF_INET6;
          infrom->sin6_port   = udp->srcport;
          *fromlen = sizeof(struct sockaddr_in6);

          net_ipv6addr_copy(infrom->sin6_addr.s6_addr, ipv6->srcipaddr);
        }
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct sockaddr_in *infrom  =
        (FAR struct sockaddr_in *)pstate->ir_from;

      if (infrom)
        {
#ifdef CONFIG_NET_IPv6
          FAR struct udp_conn_s *conn =
            (FAR struct udp_conn_s *)pstate->ir_sock->s_conn;

          /* Hybrid dual-stack IPv6/IPv4 implementations recognize a special
           * class of addresses, the IPv4-mapped IPv6 addresses.
           */

          if (conn->domain == PF_INET6)
            {
              FAR struct sockaddr_in6 *infrom6 = (FAR struct sockaddr_in6 *)infrom;
              FAR socklen_t *fromlen = pstate->ir_fromlen;
              FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
              FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
              in_addr_t ipv4addr;

              /* Encode the IPv4 address as an IPv4-mapped IPv6 address */

              infrom6->sin6_family = AF_INET6;
              infrom6->sin6_port = udp->srcport;
              *fromlen = sizeof(struct sockaddr_in6);

              ipv4addr = net_ip4addr_conv32(ipv6->srcipaddr);
              ip6_map_ipv4addr(ipv4addr, infrom6->sin6_addr.s6_addr16);
            }
          else
#endif
            {
              FAR struct udp_hdr_s *udp   = UDPIPv4BUF;
              FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

              infrom->sin_family = AF_INET;
              infrom->sin_port   = udp->srcport;

              net_ipv4addr_copy(infrom->sin_addr.s_addr,
                                net_ip4addr_conv32(ipv4->srcipaddr));
            }
        }
    }
#endif /* CONFIG_NET_IPv4 */
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_terminate
 *
 * Description:
 *   Terminate the UDP transfer.
 *
 * Input Parameters:
 *   pstate - The recvfrom state structure
 *   result - The result of the operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static void inet_udp_terminate(FAR struct inet_recvfrom_s *pstate, int result)
{
  /* Don't allow any further UDP call backs. */

  pstate->ir_cb->flags   = 0;
  pstate->ir_cb->priv    = NULL;
  pstate->ir_cb->event   = NULL;

  /* Save the result of the transfer */

  pstate->ir_result      = result;

  /* Wake up the waiting thread, returning the number of bytes
   * actually read.
   */

  nxsem_post(&pstate->ir_sem);
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   UDP receive operation via by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static uint16_t inet_udp_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvconn, FAR void *pvpriv,
                                      uint16_t flags)
{
  FAR struct inet_recvfrom_s *pstate = (FAR struct inet_recvfrom_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nerr("ERROR: Network is down\n");
          inet_udp_terminate(pstate, -ENETUNREACH);
        }

      /* If new data is available, then complete the read action. */

      else if ((flags & UDP_NEWDATA) != 0)
        {
          /* Copy the data from the packet */

          inet_udp_newdata(dev, pstate);

          /* We are finished. */

          ninfo("UDP done\n");

          /* Save the sender's address in the caller's 'from' location */

          inet_udp_sender(dev, pstate);

          /* Don't allow any further UDP call backs. */

          inet_udp_terminate(pstate, OK);

          /* Indicate that the data has been consumed */

          flags &= ~UDP_NEWDATA;
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

      else if (inet_recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          nerr("ERROR: UDP timeout\n");

          /* Terminate the transfer with an -EAGAIN error */

          inet_udp_terminate(pstate, -EAGAIN);
        }
#endif /* CONFIG_NET_SOCKOPTS */
    }

  return flags;
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_recvfrom_initialize
 *
 * Description:
 *   Initialize the state structure
 *
 * Input Parameters:
 *   psock    Pointer to the socket structure for the socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
static void inet_recvfrom_initialize(FAR struct socket *psock, FAR void *buf,
                                     size_t len, FAR struct sockaddr *infrom,
                                     FAR socklen_t *fromlen,
                                     FAR struct inet_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct inet_recvfrom_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)nxsem_init(&pstate->ir_sem, 0, 0); /* Doesn't really fail */
  (void)nxsem_setprotocol(&pstate->ir_sem, SEM_PRIO_NONE);

  pstate->ir_buflen    = len;
  pstate->ir_buffer    = buf;
  pstate->ir_from      = infrom;
  pstate->ir_fromlen   = fromlen;

  /* Set up the start time for the timeout */

  pstate->ir_sock      = psock;
#ifdef CONFIG_NET_SOCKOPTS
  pstate->ir_starttime = clock_systimer();
#endif
}

/* The only un-initialization that has to be performed is destroying the
 * semaphore.
 */

#define inet_recvfrom_uninitialize(s) nxsem_destroy(&(s)->ir_sem)

#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Input Parameters:
 *   result   The result of the net_lockedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
static ssize_t inet_recvfrom_result(int result, struct inet_recvfrom_s *pstate)
{
  /* Check for a error/timeout detected by the event handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->ir_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->ir_result;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have returned negated errno appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->ir_recvlen;
}
#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Input Parameters:
 *   psock  Pointer to the socket structure for the SOCK_DRAM socket
 *   buf    Buffer to receive data
 *   len    Length of buffer
 *   from   INET address of source (may be NULL)
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static ssize_t inet_udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                                 FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct inet_recvfrom_s state;
  int ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  inet_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

#ifdef CONFIG_NET_UDP_READAHEAD
  /* Copy the read-ahead data from the packet */

  inet_udp_readahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.ir_recvlen;

#else
  /* Otherwise, the default return value of zero is used (only for the case
   * where len == state.ir_buflen is zero).
   */

  ret = 0;
#endif

#ifdef CONFIG_NET_UDP_READAHEAD
  /* Handle non-blocking UDP sockets */

  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret < 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive anything
   * more, then we will wait to receive the data.  Otherwise return the number
   * of bytes read from the read-ahead buffer (already in 'ret').
   *
   * NOTE: that inet_udp_readahead() may set state.ir_recvlen == -1.
   */

  else if (state.ir_recvlen <= 0)
#endif
    {
      /* Get the device that will handle the packet transfers.  This may be
       * NULL if the UDP socket is bound to INADDR_ANY.  In that case, no
       * NETDEV_DOWN notifications will be received.
       */

      dev = udp_find_laddr_device(conn);

      /* Set up the callback in the connection */

      state.ir_cb = udp_callback_alloc(dev, conn);
      if (state.ir_cb)
        {
          /* Set up the callback in the connection */

          state.ir_cb->flags   = (UDP_NEWDATA | UDP_POLL | NETDEV_DOWN);
          state.ir_cb->priv    = (FAR void *)&state;
          state.ir_cb->event   = inet_udp_eventhandler;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.  net_lockedwait will also terminate if a signal is
           * received.
           */

          ret = net_lockedwait(&state. ir_sem);

          /* Make sure that no further events are processed */

          udp_callback_free(dev, conn, state.ir_cb);
          ret = inet_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock();
  inet_recvfrom_uninitialize(&state);
  return ret;
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Input Parameters:
 *   psock  Pointer to the socket structure for the SOCK_DRAM socket
 *   buf    Buffer to receive data
 *   len    Length of buffer
 *   from   INET address of source (may be NULL)
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static ssize_t inet_tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                                 FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  struct inet_recvfrom_s state;
  int               ret;

  /* Initialize the state structure.  This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  inet_recvfrom_initialize(psock, buf, len, from, fromlen, &state);

  /* Handle any any TCP data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

#ifdef CONFIG_NET_TCP_READAHEAD
  inet_tcp_readahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.ir_recvlen;

#else
  /* Otherwise, the default return value of zero is used (only for the case
   * where len == state.ir_buflen is zero).
   */

  ret = 0;
#endif

  /* Verify that the SOCK_STREAM has been and still is connected */

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      /* Was any data transferred from the readahead buffer after we were
       * disconnected?  If so, then return the number of bytes received.  We
       * will wait to return end disconnection indications the next time that
       * recvfrom() is called.
       *
       * If no data was received (i.e.,  ret == 0  -- it will not be negative)
       * and the connection was gracefully closed by the remote peer, then return
       * success.  If ir_recvlen is zero, the caller of recvfrom() will get an
       * end-of-file indication.
       */

#ifdef CONFIG_NET_TCP_READAHEAD
      if (ret <= 0 && !_SS_ISCLOSED(psock->s_flags))
#else
      if (!_SS_ISCLOSED(psock->s_flags))
#endif
        {
          /* Nothing was previously received from the readahead buffers.
           * The SOCK_STREAM must be (re-)connected in order to receive any
           * additional data.
           */

          ret = -ENOTCONN;
        }
    }

  /* In general, this implementation will not support non-blocking socket
   * operations... except in a few cases:  Here for TCP receive with read-ahead
   * enabled.  If this socket is configured as non-blocking then return EAGAIN
   * if no data was obtained from the read-ahead buffers.
   */

  else
#ifdef CONFIG_NET_TCP_READAHEAD
  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret <= 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive anything
   * more, then we will wait to receive the data.  Otherwise return the number
   * of bytes read from the read-ahead buffer (already in 'ret').
   */

  else
#endif

  /* We get here when we we decide that we need to setup the wait for incoming
   * TCP/IP data.  Just a few more conditions to check:
   *
   * 1) Make sure thet there is buffer space to receive additional data
   *    (state.ir_buflen > 0).  This could be zero, for example, if read-ahead
   *    buffering was enabled and we filled the user buffer with data from
   *    the read-ahead buffers.  And
   * 2) if read-ahead buffering is enabled (CONFIG_NET_TCP_READAHEAD)
   *    and delay logic is disabled (CONFIG_NET_TCP_RECVDELAY == 0), then we
   *    not want to wait if we already obtained some data from the read-ahead
   *    buffer.  In that case, return now with what we have (don't want for more
   *    because there may be no timeout).
   */

#if CONFIG_NET_TCP_RECVDELAY == 0 && defined(CONFIG_NET_TCP_READAHEAD)
  if (state.ir_recvlen == 0 && state.ir_buflen > 0)
#else
  if (state.ir_buflen > 0)
#endif
    {
      FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)psock->s_conn;

      /* Set up the callback in the connection */

      state.ir_cb = tcp_callback_alloc(conn);
      if (state.ir_cb)
        {
          state.ir_cb->flags   = (TCP_NEWDATA | TCP_POLL | TCP_DISCONN_EVENTS);
          state.ir_cb->priv    = (FAR void *)&state;
          state.ir_cb->event   = inet_tcp_eventhandler;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.  net_lockedwait will also terminate if a signal isi
           * received.
           */

          ret = net_lockedwait(&state.ir_sem);

          /* Make sure that no further events are processed */

          tcp_callback_free(conn, state.ir_cb);
          ret = inet_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock();
  inet_recvfrom_uninitialize(&state);
  return (ssize_t)ret;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  inet_recvfrom() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t inet_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen)
{
  ssize_t ret;

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold this address family.
   */

  if (from)
    {
      socklen_t minlen;

      /* Get the minimum socket length */

      switch (psock->s_domain)
        {
#ifdef CONFIG_NET_IPv4
        case PF_INET:
          {
            minlen = sizeof(struct sockaddr_in);
          }
          break;
#endif

#ifdef CONFIG_NET_IPv6
        case PF_INET6:
          {
            minlen = sizeof(struct sockaddr_in6);
          }
          break;
#endif

        default:
          DEBUGPANIC();
          return -EINVAL;
        }

      if (*fromlen < minlen)
        {
          return -EINVAL;
        }
    }

  /* Read from the network interface driver buffer */
  /* Or perform the TCP/IP or UDP recv() operation */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
    case SOCK_STREAM:
      {
#ifdef NET_TCP_HAVE_STACK
        ret = inet_tcp_recvfrom(psock, buf, len, from, fromlen);
#else
        ret = -ENOSYS;
#endif
      }
      break;
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
    case SOCK_DGRAM:
      {
#ifdef NET_UDP_HAVE_STACK
        ret = inet_udp_recvfrom(psock, buf, len, from, fromlen);
#else
        ret = -ENOSYS;
#endif
      }
      break;
#endif /* CONFIG_NET_UDP */

    default:
      {
        nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
        ret = -ENOSYS;
      }
      break;
    }

  return ret;
}

#endif /* CONFIG_NET */
