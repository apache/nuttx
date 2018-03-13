/****************************************************************************
 * net/tcp/tcp_timer.c
 * Poll for the availability of TCP TX data
 *
 *   Copyright (C) 2007-2010, 2015-2016, 2018 Gregory Nutt. All rights
 *     reserved.
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
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "socket/socket.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_timer
 *
 * Description:
 *   Handle a TCP timer expiration for the provided TCP connection
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *   hsed - The polling interval in halves of a second
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void tcp_timer(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
               int hsec)
{
  uint16_t result;
  uint8_t hdrlen;

  /* Set up for the callback.  We can't know in advance if the application
   * is going to send a IPv4 or an IPv6 packet, so this setup may not
   * actually be used.  Furthermore, the TCP logic is required to call
   * tcp_ipv4_select() or tcp_ipv6_select() prior to sending any packets.
   * We will try to set the correct value here basic on the binding of
   * the connection.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      hdrlen = IPv4TCP_HDRLEN;
      tcp_ipv4_select(dev);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      hdrlen = IPv6TCP_HDRLEN;
      tcp_ipv6_select(dev);
    }
#endif /* CONFIG_NET_IPv6 */

  /* Increase the TCP sequence number */

  tcp_nextsequence();

  /* Reset the length variables. */

  dev->d_len    = 0;
  dev->d_sndlen = 0;

  /* Check if the connection is in a state in which we simply wait
   * for the connection to time out. If so, we increase the
   * connection's timer and remove the connection if it times
   * out.
   */

  if (conn->tcpstateflags == TCP_TIME_WAIT ||
      conn->tcpstateflags == TCP_FIN_WAIT_2)
    {
      unsigned int newtimer;

      /* Increment the connection timer */

      newtimer = (unsigned int)conn->timer + hsec;

      /* Check if the timer exceeds the timeout value */

      if (newtimer >= TCP_TIME_WAIT_TIMEOUT)
        {
          /* Set the timer to the maximum value */

          conn->timer = TCP_TIME_WAIT_TIMEOUT;

          /* The TCP connection was established and, hence, should be bound
           * to a device. Make sure that the polling device is the one that
           * we are bound to.
           *
           * If not, then we will catch the timeout on the next poll from
           * the correct device.
           */

          DEBUGASSERT(conn->dev != NULL);
          if (dev != conn->dev)
            {
              ninfo("TCP: TCP_CLOSED pending\n");
            }
          else
            {
              conn->tcpstateflags = TCP_CLOSED;

              /* Notify upper layers about the timeout */

              result = tcp_callback(dev, conn, TCP_TIMEDOUT);

              ninfo("TCP state: TCP_CLOSED\n");
            }
        }
      else
        {
          /* No timeout. Just update the incremented timer */

          conn->timer = newtimer;
        }
    }
  else if (conn->tcpstateflags != TCP_CLOSED)
    {
      /* If the connection has outstanding data, we increase the connection's
       * timer and see if it has reached the RTO value in which case we
       * retransmit.
       */

      if (conn->unacked > 0)
        {
          /* The connection has outstanding data */

          if (conn->timer > hsec)
            {
              /* Will not yet decrement to zero */

              conn->timer -= hsec;
            }
          else
            {
              /* Will decrement to zero */

              conn->timer = 0;

              /* The TCP is connected and, hence, should be bound to a
               * device. Make sure that the polling device is the one that
               * we are bound to.
               *
               * If not, then we will catch the timeout on the next poll
               * from the correct device.
               */

              DEBUGASSERT(conn->dev != NULL);
              if (dev != conn->dev)
                {
                  ninfo("TCP: TCP_CLOSED pending\n");
                  goto done;
                }

              /* Check for a timeout on connection in the TCP_SYN_RCVD state.
               * On such timeouts, we would normally resend the SYNACK until
               * the ACK is received, completing the 3-way handshake.  But if
               * the retry count elapsed, then we must assume that no ACK is
               * forthcoming and terminate the attempted connection.
               */

              if (conn->tcpstateflags == TCP_SYN_RCVD &&
                  conn->nrtx >= TCP_MAXSYNRTX)
                {
                  FAR struct tcp_conn_s *listener;

                  conn->tcpstateflags = TCP_CLOSED;
                  ninfo("TCP state: TCP_SYN_RCVD->TCP_CLOSED\n");

                  /* Find the listener for this connection. */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
                  listener = tcp_findlistener(conn->lport, conn->domain);
#else
                  listener = tcp_findlistener(conn->lport);
#endif
                  if (listener != NULL)
                    {
                      /* We call tcp_callback() for the connection with
                       * TCP_TIMEDOUT to inform the listener that the
                       * connection has timed out.
                       */

                      result = tcp_callback(dev, listener, TCP_TIMEDOUT);
                    }

                  /* We also send a reset packet to the remote host. */

                  tcp_send(dev, conn, TCP_RST | TCP_ACK, hdrlen);

                  /* Finally, we must free this TCP connection structure */

                  tcp_free(conn);
                  goto done;
                }

              /* Otherwise, check for a timeout on an established connection.
               * If the retry count is exceeded in this case, we should
               * close the connection.
               */

              else if (
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
                  conn->expired > 0 ||
#else
                  conn->nrtx >= TCP_MAXRTX ||
#endif
                  (conn->tcpstateflags == TCP_SYN_SENT &&
                   conn->nrtx >= TCP_MAXSYNRTX)
                 )
                {
                  conn->tcpstateflags = TCP_CLOSED;
                  ninfo("TCP state: TCP_CLOSED\n");

                  /* We call tcp_callback() with TCP_TIMEDOUT to
                   * inform the application that the connection has
                   * timed out.
                   */

                  result = tcp_callback(dev, conn, TCP_TIMEDOUT);

                  /* We also send a reset packet to the remote host. */

                  tcp_send(dev, conn, TCP_RST | TCP_ACK, hdrlen);
                  goto done;
                }

             /* Exponential backoff. */

              conn->timer = TCP_RTO << (conn->nrtx > 4 ? 4: conn->nrtx);
              (conn->nrtx)++;

              /* Ok, so we need to retransmit. We do this differently
               * depending on which state we are in. In ESTABLISHED, we
               * call upon the application so that it may prepare the
               * data for the retransmit. In SYN_RCVD, we resend the
               * SYNACK that we sent earlier and in LAST_ACK we have to
               * retransmit our FINACK.
               */

#ifdef CONFIG_NET_STATISTICS
              g_netstats.tcp.rexmit++;
#endif
              switch (conn->tcpstateflags & TCP_STATE_MASK)
                {
                  case TCP_SYN_RCVD:
                    /* In the SYN_RCVD state, we should retransmit our
                     * SYNACK.
                     */

                    tcp_ack(dev, conn, TCP_ACK | TCP_SYN);
                    goto done;

                  case TCP_SYN_SENT:
                    /* In the SYN_SENT state, we retransmit out SYN. */

                    tcp_ack(dev, conn, TCP_SYN);
                    goto done;

                  case TCP_ESTABLISHED:
                    /* In the ESTABLISHED state, we call upon the application
                     * to do the actual retransmit after which we jump into
                     * the code for sending out the packet.
                     */

                    result = tcp_callback(dev, conn, TCP_REXMIT);
                    tcp_rexmit(dev, conn, result);
                    goto done;

                  case TCP_FIN_WAIT_1:
                  case TCP_CLOSING:
                  case TCP_LAST_ACK:
                    /* In all these states we should retransmit a FINACK. */

                    tcp_send(dev, conn, TCP_FIN | TCP_ACK, hdrlen);
                    goto done;
                }
            }
        }

      /* The connection does not have outstanding data.  Check if the TCP
       * connection has been established.
       */

      else if ((conn->tcpstateflags & TCP_STATE_MASK) == TCP_ESTABLISHED)
        {
          /* The TCP connection is established and, hence, should be bound
           * to a device. Make sure that the polling device is the one that
           * we are bound to.
           */

          DEBUGASSERT(conn->dev != NULL);
          if (dev == conn->dev)
            {
#ifdef CONFIG_NET_TCP_KEEPALIVE
              /* Is this an established connected with KeepAlive enabled? */

              if (conn->keepalive)
                {
                  socktimeo_t timeo;
                  uint32_t saveseq;

                  /* If this is the first probe, then the keepstart time is
                   * the time that the last ACK or data was received from the
                   * remote.
                   *
                   * On subsequent retries, keepstart is the time that the
                   * last probe was sent.
                   */

                  if (conn->keepretries > 0)
                    {
                      timeo = (socktimeo_t)conn->keepintvl;
                    }
                  else
                    {
                      timeo = (socktimeo_t)conn->keepidle;
                    }

                  /* Yes... has the idle period elapsed with no data or ACK
                   * received from the remote peer?
                   */

                  if (net_timeo(conn->keeptime, timeo))
                    {
                      /* Yes.. Has the retry count expired? */

                      if (conn->keepretries >= conn->keepcnt)
                        {
                          /* Yes... stop the network monitor, closing the connection and all sockets
                           * associated with the connection.
                           */

                          tcp_stop_monitor(conn, TCP_ABORT);
                        }
                      else
                        {
                          unsigned int tcpiplen;

                          /* No.. we need to send another probe.
                           *
                           * Get the size of the IP header and the TCP header.
                           */
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
                          if (conn->domain == PF_INET)
#endif
                            {
                              tcpiplen = IPv4_HDRLEN + TCP_HDRLEN;
                            }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
                          else
#endif
                            {
                              tcpiplen = IPv6_HDRLEN + TCP_HDRLEN;
                            }
#endif

                          /* And send the probe (along with a garbage byte).
                           * The packet we sned must have these properties:
                           *
                           *   - TCP_ACK flag (only) is set.
                           *   - Sequence number is the sequence number of
                           *     previously ACKed data, i.e., the expected
                           *     sequence number minus one.
                           *   - The data payload is one or two bytes.
                           *
                           * tcp_send() will send the TCP sequence number as
                           * conn->sndseq.  Rather than creating a new
                           * interface, we spoof tcp_end() here:
                           */

                          saveseq = tcp_getsequence(conn->sndseq);
                          tcp_setsequence(conn->sndseq, saveseq - 1);

                          tcp_send(dev, conn, TCP_ACK, tcpiplen + 1);

                          tcp_setsequence(conn->sndseq, saveseq);

                          /* Increment the number of un-ACKed bytes due to the dummy
                           * byte that we just sent.
                           */

                          conn->unacked++;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
                          /* Increment the un-ACKed sequence number */

                          conn->sndseq_max++;
#endif
                          /* Update for the next probe */

                          conn->keeptime = clock_systimer();
                          conn->keepretries++;
                        }

                      goto done;
                    }
                }
#endif
              /* There was no need for a retransmission and there was no
               * need to probe the remote peer.  We poll the application for
               * new outgoing data.
               */

              result = tcp_callback(dev, conn, TCP_POLL);
              tcp_appsend(dev, conn, result);
              goto done;
            }
        }
    }

  /* Nothing to be done */

  dev->d_len = 0;

done:
  return;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
