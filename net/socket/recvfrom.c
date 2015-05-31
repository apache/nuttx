/****************************************************************************
 * net/socket/recvfrom.c
 *
 *   Copyright (C) 2007-2009, 2011-2015 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET_PKT
#  include <netpacket/packet.h>
#endif

#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/iob.h>
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

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
struct recvfrom_s
{
  FAR struct socket       *rf_sock;      /* The parent socket structure */
#ifdef CONFIG_NET_SOCKOPTS
  uint32_t                 rf_starttime; /* rcv start time for determining timeout */
#endif
  FAR struct devif_callback_s *rf_cb;    /* Reference to callback instance */
  sem_t                    rf_sem;       /* Semaphore signals recv completion */
  size_t                   rf_buflen;    /* Length of receive buffer */
  uint8_t                 *rf_buffer;    /* Pointer to receive buffer */
  FAR struct sockaddr     *rf_from;      /* Address of sender */
  FAR socklen_t           *rf_fromlen;   /* Number of bytes allocated for address of sender */
  size_t                   rf_recvlen;   /* The received length */
  int                      rf_result;    /* Success:OK, failure:negated errno */
};
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   The number of bytes taken from the packet.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static size_t recvfrom_newdata(FAR struct net_driver_s *dev,
                               FAR struct recvfrom_s *pstate)
{
  size_t recvlen;

  /* Get the length of the data to return */

  if (dev->d_len > pstate->rf_buflen)
    {
      recvlen = pstate->rf_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new appdata into the user buffer */

  memcpy(pstate->rf_buffer, dev->d_appdata, recvlen);
  nllvdbg("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  pstate->rf_recvlen += recvlen;
  pstate->rf_buffer  += recvlen;
  pstate->rf_buflen  -= recvlen;

  return recvlen;
}
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_newpktdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static void recvfrom_newpktdata(FAR struct net_driver_s *dev,
                                FAR struct recvfrom_s *pstate)
{
  size_t recvlen;

  if (dev->d_len > pstate->rf_buflen)
    {
      recvlen = pstate->rf_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new packet data into the user buffer */

  memcpy(pstate->rf_buffer, dev->d_buf, recvlen);
  nllvdbg("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  pstate->rf_recvlen += recvlen;
  pstate->rf_buffer  += recvlen;
  pstate->rf_buffer  -= recvlen;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Function: recvfrom_newtcpdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void recvfrom_newtcpdata(FAR struct net_driver_s *dev,
                                       FAR struct recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  size_t recvlen = recvfrom_newdata(dev, pstate);

  /* If there is more data left in the packet that we could not buffer, than
   * add it to the read-ahead buffers.
   */

 if (recvlen < dev->d_len)
   {
#ifdef CONFIG_NET_TCP_READAHEAD
      FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pstate->rf_sock->s_conn;
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
          ndbg("ERROR: packet data not saved (%d bytes)\n", buflen - nsaved);
        }
#endif
#else
      ndbg("ERROR: packet data lost (%d bytes)\n", dev->d_len - recvlen);
#endif
   }

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_newudpdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void recvfrom_newudpdata(FAR struct net_driver_s *dev,
                                       FAR struct recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  (void)recvfrom_newdata(dev, pstate);

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_tcpreadahead
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_READAHEAD)
static inline void recvfrom_tcpreadahead(struct recvfrom_s *pstate)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pstate->rf_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any TCP data already buffered in a read-ahead
   * buffer.
   */

  while ((iob = iob_peek_queue(&conn->readahead)) != NULL &&
          pstate->rf_buflen > 0)
    {
      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(pstate->rf_buffer, iob, pstate->rf_buflen, 0);
      nllvdbg("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

      /* Update the accumulated size of the data read */

      pstate->rf_recvlen += recvlen;
      pstate->rf_buffer  += recvlen;
      pstate->rf_buflen  -= recvlen;

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
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

#if defined(CONFIG_NET_UDP) && defined(CONFIG_NET_UDP_READAHEAD)

static inline void recvfrom_udpreadahead(struct recvfrom_s *pstate)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)pstate->rf_sock->s_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any UDP datagram already buffered in a read-ahead
   * buffer.
   */

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL &&
          pstate->rf_buflen > 0)
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

      if ( 0
#ifdef CONFIG_NET_IPv6
           || src_addr_size == sizeof(struct sockaddr_in6)
#endif
#ifdef CONFIG_NET_IPv4
           || src_addr_size == sizeof(struct sockaddr_in)
#endif
        )
        {
          if (pstate->rf_from)
            {
              socklen_t len = *pstate->rf_fromlen;
              len = (socklen_t)src_addr_size > len ? len : (socklen_t)src_addr_size;

              recvlen = iob_copyout((FAR uint8_t *)pstate->rf_from, iob,
                                    len, sizeof(uint8_t));
              if (recvlen != len)
                {
                  goto out;
                }
            }
        }

      recvlen = iob_copyout(pstate->rf_buffer, iob, pstate->rf_buflen,
                            src_addr_size + sizeof(uint8_t));

      nllvdbg("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

      /* Update the accumulated size of the data read */

      pstate->rf_recvlen += recvlen;
      pstate->rf_buffer  += recvlen;
      pstate->rf_buflen  -= recvlen;

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
 * Function: recvfrom_timeout
 *
 * Description:
 *   Check for recvfrom timeout.
 *
 * Parameters:
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
#ifdef CONFIG_NET_SOCKOPTS
static int recvfrom_timeout(struct recvfrom_s *pstate)
{
  FAR struct socket *psock = 0;
  socktimeo_t        timeo = 0;

  /* Check for a timeout configured via setsockopts(SO_RCVTIMEO). If none...
   * we well let the read hang forever (except for the special case below).
   */

  /* Get the socket reference from the private data */

  psock = pstate->rf_sock;
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
      pstate->rf_recvlen > 0)
    {
      /* Use the configured timeout */

      timeo = CONFIG_NET_TCP_RECVDELAY;
    }
#endif

  /* Is there an effective timeout? */

  if (timeo)
    {
      /* Yes.. Check if the timeout has elapsed */

      return net_timeo(pstate->rf_starttime, timeo);
    }

  /* No timeout -- hang forever waiting for data. */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS */
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_pktsender
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Values:
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static inline void recvfrom_pktsender(FAR struct net_driver_s *dev,
                                      FAR struct recvfrom_s *pstate)
{
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Function: recvfrom_pktinterrupt
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Values:
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static uint16_t recvfrom_pktinterrupt(FAR struct net_driver_s *dev,
                                      FAR void *conn, FAR void *pvpriv,
                                      uint16_t flags)
{
  struct recvfrom_s *pstate = (struct recvfrom_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If a new packet is available, then complete the read action. */

      if ((flags & PKT_NEWDATA) != 0)
        {
          /* Copy the packet */
          recvfrom_newpktdata(dev, pstate);

          /* We are finished. */

          nllvdbg("PKT done\n");

          /* Don't allow any further call backs. */

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;
#if 0
          /* Save the sender's address in the caller's 'from' location */

          recvfrom_pktsender(dev, pstate);
#endif
          /* indicate that the data has been consumed */

          flags &= ~PKT_NEWDATA;

          /* Wake up the waiting thread, returning the number of bytes
           * actually read.
           */

          sem_post(&pstate->rf_sem);
        }
    }

  return flags;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Function: recvfrom_tcpsender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void recvfrom_tcpsender(FAR struct net_driver_s *dev,
                                      FAR struct recvfrom_s *pstate)
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
        (FAR struct sockaddr_in6 *)pstate->rf_from;

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
        (FAR struct sockaddr_in *)pstate->rf_from;

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
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_tcpinterrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   TCP receive operation via by the lower, device interfacing layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static uint16_t recvfrom_tcpinterrupt(FAR struct net_driver_s *dev,
                                      FAR void *conn, FAR void *pvpriv,
                                      uint16_t flags)
{
  FAR struct recvfrom_s *pstate = (struct recvfrom_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If new data is available, then complete the read action. */

      if ((flags & TCP_NEWDATA) != 0)
        {
          /* Copy the data from the packet (saving any unused bytes from the
           * packet in the read-ahead buffer).
           */

          recvfrom_newtcpdata(dev, pstate);

          /* Save the sender's address in the caller's 'from' location */

          recvfrom_tcpsender(dev, pstate);

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
          if (pstate->rf_buflen == 0)
#else
          if (pstate->rf_recvlen > 0)
#endif
            {
              nllvdbg("TCP resume\n");

              /* The TCP receive buffer is full.  Return now and don't allow
               * any further TCP call backs.
               */

              pstate->rf_cb->flags   = 0;
              pstate->rf_cb->priv    = NULL;
              pstate->rf_cb->event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              sem_post(&pstate->rf_sem);
            }

#ifdef CONFIG_NET_SOCKOPTS
            /* Reset the timeout.  We will want a short timeout to terminate
             * the TCP receive.
             */

            pstate->rf_starttime = clock_systimer();
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
          nllvdbg("Lost connection\n");

          /* Stop further callbacks */

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Handle loss-of-connection event */

          net_lostconnection(pstate->rf_sock, flags);

          /* Check if the peer gracefully closed the connection. */

          if ((flags & TCP_CLOSE) != 0)
            {
              /* This case should always return success (zero)! The value of
               * rf_recvlen, if zero, will indicate that the connection was
               * gracefully closed.
               */

              pstate->rf_result = 0;
            }
          else
            {
              /* If no data has been received, then return ENOTCONN.
               * Otherwise, let this return success.  The failure will
               * be reported the next time that recv[from]() is called.
               */

#if CONFIG_NET_TCP_RECVDELAY > 0
              if (pstate->rf_recvlen > 0)
                {
                  pstate->rf_result = 0;
                }
              else
                {
                  pstate->rf_result = -ENOTCONN;
                }
#else
              pstate->rf_result = -ENOTCONN;
#endif
            }

          /* Wake up the waiting thread */

          sem_post(&pstate->rf_sem);
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

      else if (recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          nllvdbg("TCP timeout\n");

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Report an error only if no data has been received. (If
           * CONFIG_NET_TCP_RECVDELAY then rf_recvlen should always be
           * zero).
           */

#if CONFIG_NET_TCP_RECVDELAY > 0
          if (pstate->rf_recvlen == 0)
#endif
            {
              /* Report the timeout error */

              pstate->rf_result = -EAGAIN;
            }

          /* Wake up the waiting thread, returning either the error -EAGAIN
           * that signals the timeout event or the data received up to
           * the point that the timeout occurred (no error).
           */

          sem_post(&pstate->rf_sem);
        }
#endif /* CONFIG_NET_SOCKOPTS */
    }

  return flags;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_udpsender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void recvfrom_udpsender(struct net_driver_s *dev, struct recvfrom_s *pstate)
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
        (FAR struct sockaddr_in6 *)pstate->rf_from;
      FAR socklen_t *fromlen = pstate->rf_fromlen;

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
        (FAR struct sockaddr_in *)pstate->rf_from;

      if (infrom)
        {
#ifdef CONFIG_NET_IPv6
          FAR struct udp_conn_s *conn = (FAR struct udp_conn_s*)pstate->rf_sock->s_conn;

          /* Hybrid dual-stack IPv6/IPv4 implementations recognize a special
           * class of addresses, the IPv4-mapped IPv6 addresses.
           */

          if (conn->domain == PF_INET6)
            {
              FAR struct sockaddr_in6 *infrom6 = (FAR struct sockaddr_in6 *)infrom;
              FAR socklen_t *fromlen = pstate->rf_fromlen;
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
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: recvfrom_udp_terminate
 *
 * Description:
 *   Terminate the UDP transfer.
 *
 * Parameters:
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static void recvfrom_udp_terminate(FAR struct recvfrom_s *pstate, int result)
{
  /* Don't allow any further UDP call backs. */

  pstate->rf_cb->flags   = 0;
  pstate->rf_cb->priv    = NULL;
  pstate->rf_cb->event   = NULL;

  /* Save the result of the transfer */

  pstate->rf_result      = result;

  /* Wake up the waiting thread, returning the number of bytes
   * actually read.
   */

  sem_post(&pstate->rf_sem);
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: recvfrom_udp_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   UDP receive operation via by the lower, device interfacing layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static uint16_t recvfrom_udp_interrupt(FAR struct net_driver_s *dev,
                                       FAR void *pvconn, FAR void *pvpriv,
                                       uint16_t flags)
{
  FAR struct recvfrom_s *pstate = (FAR struct recvfrom_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nlldbg("ERROR: Network is down\n");
          recvfrom_udp_terminate(pstate, -ENETUNREACH);
        }

      /* If new data is available, then complete the read action. */

      else if ((flags & UDP_NEWDATA) != 0)
        {
          /* Copy the data from the packet */

          recvfrom_newudpdata(dev, pstate);

          /* We are finished. */

          nllvdbg("UDP done\n");

          /* Save the sender's address in the caller's 'from' location */

          recvfrom_udpsender(dev, pstate);

          /* Don't allow any further UDP call backs. */

          recvfrom_udp_terminate(pstate, OK);

          /* Indicate that the data has been consumed */

          flags &= ~UDP_NEWDATA;
        }

#ifdef CONFIG_NET_SOCKOPTS
      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

      else if (recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          nllvdbg("ERROR: UDP timeout\n");

          /* Terminate the transfer with an -EAGAIN error */

          recvfrom_udp_terminate(pstate, -EAGAIN);
        }
#endif /* CONFIG_NET_SOCKOPTS */
    }

  return flags;
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: recvfrom_init
 *
 * Description:
 *   Initialize the state structure
 *
 * Parameters:
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

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static void recvfrom_init(FAR struct socket *psock, FAR void *buf,
                          size_t len, FAR struct sockaddr *infrom,
                          FAR socklen_t *fromlen,
                          FAR struct recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct recvfrom_s));
  (void)sem_init(&pstate->rf_sem, 0, 0); /* Doesn't really fail */
  pstate->rf_buflen    = len;
  pstate->rf_buffer    = buf;
  pstate->rf_from      = infrom;
  pstate->rf_fromlen   = fromlen;

  /* Set up the start time for the timeout */

  pstate->rf_sock      = psock;
#ifdef CONFIG_NET_SOCKOPTS
  pstate->rf_starttime = clock_systimer();
#endif
}

/* The only un-initialization that has to be performed is destroying the
 * semaphore.
 */

#define recvfrom_uninit(s) sem_destroy(&(s)->rf_sem)

#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Parameters:
 *   result   The result of the net_lockedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static ssize_t recvfrom_result(int result, struct recvfrom_s *pstate)
{
  int save_errno = errno; /* In case something we do changes it */

  /* Check for a error/timeout detected by the interrupt handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->rf_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->rf_result;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have set errno appropriately.
   */

  if (result < 0)
    {
      return -save_errno;
    }

  return pstate->rf_recvlen;
}
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfromo_pkt_rxnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are ready to receive a
 *   packet (PKT)
 *
 * Parameters:
 *   conn - The PKT connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not implemented */
static void recvfromo_pkt_rxnotify(FAR struct pkt_conn_s *conn)
{
#  warning Missing logic
}
#endif

/****************************************************************************
 * Function: recvfrom_udp_rxnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are ready to receive a
 *   packet (UDP)
 *
 * Parameters:
 *   psock - Socket state structure
 *   conn  - The UDP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void recvfrom_udp_rxnotify(FAR struct socket *psock,
                                         FAR struct udp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver of the receive ready */

#ifdef CONFIG_NETDEV_MULTINIC
      netdev_ipv4_rxnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
#else
      netdev_ipv4_rxnotify(conn->u.ipv4.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver of the receive ready */

      DEBUGASSERT(psock->s_domain == PF_INET6);
#ifdef CONFIG_NETDEV_MULTINIC
      netdev_ipv6_rxnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
#else
      netdev_ipv6_rxnotify(conn->u.ipv6.raddr);
#endif
    }
#endif /* CONFIG_NET_IPv6 */
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: pkt_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for packet socket
 *
 * Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static ssize_t pkt_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct pkt_conn_s *conn = (FAR struct pkt_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct recvfrom_s state;
  net_lock_t save;
  int ret;

  /* Perform the packet recvfrom() operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = net_lock();
  recvfrom_init(psock, buf, len, from, fromlen, &state);

  /* Get the device driver that will service this transfer */

  dev  = pkt_find_device(conn);
  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout_with_state;
    }

  /* TODO recvfrom_init() expects from to be of type sockaddr_in, but
   * in our case is sockaddr_ll
   */

#if 0
  ret = pkt_connect(conn, NULL);
  if (ret < 0)
    {
      goto errout_with_state;
    }
#endif

  /* Set up the callback in the connection */

  state.rf_cb = pkt_callback_alloc(dev, conn);
  if (state.rf_cb)
    {
      state.rf_cb->flags  = (PKT_NEWDATA | PKT_POLL);
      state.rf_cb->priv   = (void*)&state;
      state.rf_cb->event  = recvfrom_pktinterrupt;

      /* Notify the device driver of the receive call */

#if 0 /* Not implemented */
      recvfromo_pkt_rxnotify(conn);
#endif

      /* Wait for either the receive to complete or for an error/timeout to occur.
       * NOTES:  (1) net_lockedwait will also terminate if a signal is received, (2)
       * interrupts are disabled!  They will be re-enabled while the task sleeps
       * and automatically re-enabled when the task restarts.
       */

      ret = net_lockedwait(&state.rf_sem);

      /* Make sure that no further interrupts are processed */

      pkt_callback_free(dev, conn, state.rf_cb);
      ret = recvfrom_result(ret, &state);
    }
  else
    {
      ret = -EBUSY;
    }

errout_with_state:
  net_unlock(save);
  recvfrom_uninit(&state);
  return ret;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Function: udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Parameters:
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

#ifdef CONFIG_NET_UDP
static ssize_t udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct recvfrom_s state;
  net_lock_t save;
  int ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = net_lock();
  recvfrom_init(psock, buf, len, from, fromlen, &state);

  /* Setup the UDP remote connection */

  ret = udp_connect(conn, NULL);
  if (ret < 0)
    {
      goto errout_with_state;
    }

#ifdef CONFIG_NET_UDP_READAHEAD
  recvfrom_udpreadahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.rf_recvlen;

#else
  /* Otherwise, the default return value of zero is used (only for the case
   * where len == state.rf_buflen is zero).
   */

  ret = 0;
#endif

#ifdef CONFIG_NET_UDP_READAHEAD
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

  else if (state.rf_recvlen == 0)
#endif
    {
      /* Get the device that will handle the packet transfers.  This may be
       * NULL if the UDP socket is bound to INADDR_ANY.  In that case, no
       * NETDEV_DOWN notifications will be received.
       */

      dev = udp_find_laddr_device(conn);

      /* Set up the callback in the connection */

      state.rf_cb = udp_callback_alloc(dev, conn);
      if (state.rf_cb)
        {
          /* Set up the callback in the connection */

          state.rf_cb->flags   = (UDP_NEWDATA | UDP_POLL | NETDEV_DOWN);
          state.rf_cb->priv    = (void*)&state;
          state.rf_cb->event   = recvfrom_udp_interrupt;

          /* Notify the device driver of the receive call */

          recvfrom_udp_rxnotify(psock, conn);

          /* Wait for either the receive to complete or for an error/timeout
           * to occur. NOTES:  (1) net_lockedwait will also terminate if a
           * signal is received, (2) interrupts are disabled!  They will be
           * re-enabled while the task sleeps and automatically re-enabled
           * when the task restarts.
           */

          ret = net_lockedwait(&state. rf_sem);

          /* Make sure that no further interrupts are processed */

          udp_callback_free(dev, conn, state.rf_cb);
          ret = recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

errout_with_state:
  net_unlock(save);
  recvfrom_uninit(&state);
  return ret;
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Parameters:
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

#ifdef CONFIG_NET_TCP
static ssize_t tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  struct recvfrom_s       state;
  net_lock_t              save;
  int                     ret;

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = net_lock();
  recvfrom_init(psock, buf, len, from, fromlen, &state);

  /* Handle any any TCP data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

#ifdef CONFIG_NET_TCP_READAHEAD
  recvfrom_tcpreadahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.rf_recvlen;

#else
  /* Otherwise, the default return value of zero is used (only for the case
   * where len == state.rf_buflen is zero).
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
       * success.  If rf_recvlen is zero, the caller of recvfrom() will get an
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

  /* In general, this uIP-based implementation will not support non-blocking
   * socket operations... except in a few cases:  Here for TCP receive with read-ahead
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
   *    (state.rf_buflen > 0).  This could be zero, for example, if read-ahead
   *    buffering was enabled and we filled the user buffer with data from
   *    the read-ahead buffers.  And
   * 2) if read-ahead buffering is enabled (CONFIG_NET_TCP_READAHEAD)
   *    and delay logic is disabled (CONFIG_NET_TCP_RECVDELAY == 0), then we
   *    not want to wait if we already obtained some data from the read-ahead
   *    buffer.  In that case, return now with what we have (don't want for more
   *    because there may be no timeout).
   */

#if CONFIG_NET_TCP_RECVDELAY == 0 && defined(CONFIG_NET_TCP_READAHEAD)
  if (state.rf_recvlen == 0 && state.rf_buflen > 0)
#else
  if (state.rf_buflen > 0)
#endif
    {
      FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)psock->s_conn;

      /* Set up the callback in the connection */

      state.rf_cb = tcp_callback_alloc(conn);
      if (state.rf_cb)
        {
          state.rf_cb->flags   = (TCP_NEWDATA | TCP_POLL | TCP_DISCONN_EVENTS);
          state.rf_cb->priv    = (void*)&state;
          state.rf_cb->event   = recvfrom_tcpinterrupt;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.
           *
           * NOTES:  (1) net_lockedwait will also terminate if a signal is
           * received, (2) interrupts may be disabled!  They will be re-
           * enabled while the task sleeps and automatically re-enabled when
           * the task restarts.
           */

          ret = net_lockedwait(&state.rf_sem);

          /* Make sure that no further interrupts are processed */

          tcp_callback_free(conn, state.rf_cb);
          ret = recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock(save);
  recvfrom_uninit(&state);
  return (ssize_t)ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
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
 *   recv() will return 0.  Otherwise, on errors, -1 is returned, and errno
 *   is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                       int flags, FAR struct sockaddr *from,
                       FAR socklen_t *fromlen)
{
  ssize_t ret;
  int err;

  /* Verify that non-NULL pointers were passed */

#ifdef CONFIG_DEBUG
  if (!buf)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  if (from && !fromlen)
    {
      err = EINVAL;
      goto errout;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

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

#ifdef CONFIG_NET_LOCAL
        case PF_LOCAL:
          {
            minlen = sizeof(sa_family_t);
          }
          break;
#endif

        default:
          DEBUGPANIC();
          err = EINVAL;
          goto errout;
        }

      if (*fromlen < minlen)
        {
          err = EINVAL;
          goto errout;
        }
    }

  /* Set the socket state to receiving */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_RECV);

  /* Read from the network interface driver buffer */
  /* Or perform the TCP/IP or UDP recv() operation */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_PKT
    case SOCK_RAW:
      {
        ret = pkt_recvfrom(psock, buf, len, from, fromlen);
      }
      break;
#endif /* CONFIG_NET_PKT */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
    case SOCK_STREAM:
      {
#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
        if (psock->s_domain == PF_LOCAL)
#endif
          {
            ret = psock_local_recvfrom(psock, buf, len, flags,
                                       from, fromlen);
          }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
        else
#endif
          {
            ret = tcp_recvfrom(psock, buf, len, from, fromlen);
          }
#endif /* CONFIG_NET_TCP */
      }
      break;
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM */

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
    case SOCK_DGRAM:
      {
#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
        if (psock->s_domain == PF_LOCAL)
#endif
          {
            ret = psock_local_recvfrom(psock, buf, len, flags,
                                       from, fromlen);
          }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
        else
#endif
          {
            ret = udp_recvfrom(psock, buf, len, from, fromlen);
          }
#endif /* CONFIG_NET_UDP */
      }
      break;
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL_DGRAM */

    default:
      {
        ndbg("ERROR: Unsupported socket type: %d\n", psock->s_type);
        ret = -ENOSYS;
      }
      break;
    }

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Handle returned errors */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Success return */

  return ret;

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t recvfrom(int sockfd, FAR void *buf, size_t len, int flags,
                 FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_recvfrom() do all of the work */

  return psock_recvfrom(psock, buf, len, flags, from, fromlen);
}

#endif /* CONFIG_NET */
