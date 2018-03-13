/****************************************************************************
 * net/tcp/tcp_appsend.c
 *
 *   Copyright (C) 2007-2010, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_appsend
 *
 * Description:
 *   Handle application or TCP protocol response.  If this function is called
 *   with dev->d_sndlen > 0, then this is an application attempting to send
 *   packet.
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void tcp_appsend(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                 uint16_t result)
{
  uint8_t hdrlen;

  /* Handle the result based on the application response */

  ninfo("result: %04x d_sndlen: %d conn->unacked: %d\n",
        result, dev->d_sndlen, conn->unacked);

  /* Get the IP header length associated with the IP domain configured for
   * this TCP connection.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      DEBUGASSERT(IFF_IS_IPv4(dev->d_flags));
      hdrlen = IPv4TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      DEBUGASSERT(IFF_IS_IPv6(dev->d_flags));
      hdrlen = IPv6TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv6 */

  /* Check If the device went down */

  if ((result & NETDEV_DOWN) != 0)
    {
      /* If so, make sure that the connection is marked closed
       * and do not try to send anything.
       */

      dev->d_sndlen = 0;
      conn->tcpstateflags = TCP_CLOSED;
      ninfo("TCP state: NETDEV_DOWN\n");
    }

  /* Check for connection aborted */

  else if ((result & TCP_ABORT) != 0)
    {
      dev->d_sndlen = 0;
      conn->tcpstateflags = TCP_CLOSED;
      ninfo("TCP state: TCP_CLOSED\n");

      tcp_send(dev, conn, TCP_RST | TCP_ACK, hdrlen);
    }

  /* Check for connection closed */

  else if ((result & TCP_CLOSE) != 0)
    {
      conn->tcpstateflags = TCP_FIN_WAIT_1;
      conn->unacked  = 1;
      conn->nrtx     = 0;
      ninfo("TCP state: TCP_FIN_WAIT_1\n");

      dev->d_sndlen  = 0;
      tcp_send(dev, conn, TCP_FIN | TCP_ACK, hdrlen);
    }

  /* None of the above */

  else
    {
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      DEBUGASSERT(dev->d_sndlen <= conn->mss);
#else
      /* If d_sndlen > 0, the application has data to be sent. */

      if (dev->d_sndlen > 0)
        {
          /* Remember how much data we send out now so that we know
           * when everything has been acknowledged.  Just increment the amount
           * of data sent.  This will be needed in sequence number calculations
           * and we know that this is not a re-transmission.  Retransmissions
           * do not go through this path.
           */

          conn->unacked += dev->d_sndlen;

          /* The application cannot send more than what is allowed by the
           * MSS (the minumum of the MSS and the available window).
           */

          DEBUGASSERT(dev->d_sndlen <= conn->mss);
        }

      conn->nrtx = 0;
#endif
      /* Then handle the rest of the operation just as for the rexmit case */

      tcp_rexmit(dev, conn, result);
    }
}

/****************************************************************************
 * Name: tcp_rexmit
 *
 * Description:
 *   Handle application retransmission
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void tcp_rexmit(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint16_t result)
{
  uint8_t hdrlen;

  ninfo("result: %04x d_sndlen: %d conn->unacked: %d\n",
        result, dev->d_sndlen, conn->unacked);

  /* Get the IP header length associated with the IP domain configured for
   * this TCP connection.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      DEBUGASSERT(IFF_IS_IPv4(dev->d_flags));
      hdrlen = IPv4TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      DEBUGASSERT(IFF_IS_IPv6(dev->d_flags));
      hdrlen = IPv6TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv6 */

  /* If the application has data to be sent, or if the incoming packet had
   * new data in it, we must send out a packet.
   */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  if (dev->d_sndlen > 0)
#else
  if (dev->d_sndlen > 0 && conn->unacked > 0)
#endif
    {
      /* We always set the ACK flag in response packets adding the length of
       * the IP and TCP headers.
       */

      tcp_send(dev, conn, TCP_ACK | TCP_PSH, dev->d_sndlen + hdrlen);
    }

  /* If there is no data to send, just send out a pure ACK if one is requested`. */

  else if ((result & TCP_SNDACK) != 0)
    {
      tcp_send(dev, conn, TCP_ACK, hdrlen);
    }

  /* There is nothing to do -- drop the packet */

  else
    {
      dev->d_len = 0;
    }
}
#endif /* CONFIG_NET && CONFIG_NET_TCP */
