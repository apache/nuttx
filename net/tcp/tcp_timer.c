/****************************************************************************
 * net/tcp/tcp_timer.c
 * Poll for the availability of TCP TX data
 *
 *   Copyright (C) 2007-2010, 2015-2016, 2018, 2020 Gregory Nutt. All rights
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
#include <time.h>
#include <stdlib.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per RFC 1122:  "... an ACK should not be excessively delayed; in
 * particular, the delay MUST be less than 0.5 seconds ..."
 *
 * NOTE:  We only have 0.5 timing resolution here so the delay will be
 * between 0.5 and 1.0 seconds, and may be delayed further, depending on the
 * polling rate of the the driver (often 1 second).
 */

#define ACK_DELAY (1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_get_timeout
 *
 * Description:
 *   Gets the time of the next timeout
 *
 * Input Parameters:
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   int - The time required for the next expiry (units: half-seconds)
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

static int tcp_get_timeout(FAR struct tcp_conn_s *conn)
{
  int timeout = conn->timer;

#ifdef CONFIG_NET_TCP_KEEPALIVE
  if (timeout == 0)
    {
      timeout = conn->keeptimer;
    }
  else if (conn->keeptimer > 0 && timeout > conn->keeptimer)
    {
      timeout = conn->keeptimer;
    }
#endif

  return timeout;
}

/****************************************************************************
 * Name: tcp_timer_expiry
 *
 * Description:
 *   Handle a TCP timer expiration for the provided TCP connection
 *   Restart a TCP timer if need to
 *
 * Input Parameters:
 *   arg - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   arg is not NULL.
 *   The connection (arg) is bound to the polling device (dev).
 *
 ****************************************************************************/

static void tcp_timer_expiry(FAR void *arg)
{
  FAR struct tcp_conn_s *conn = NULL;

  net_lock();

  while ((conn = tcp_nextconn(conn)) != NULL)
    {
      if (conn == arg)
        {
          conn->timeout = true;
          netdev_txnotify_dev(conn->dev);
          break;
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: tcp_update_timer
 *
 * Description:
 *   Update the TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

static void tcp_update_timer(FAR struct tcp_conn_s *conn)
{
  int timeout = tcp_get_timeout(conn);

  if (timeout > 0)
    {
      if (TICK2HSEC(work_timeleft(&conn->work)) != timeout)
        {
          work_queue(LPWORK, &conn->work, tcp_timer_expiry,
                     conn, HSEC2TICK(timeout));
        }
    }
  else
    {
      work_cancel(LPWORK, &conn->work);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_update_retrantimer
 *
 * Description:
 *   Update the retransmit TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn    - The TCP "connection" to poll for TX data
 *   timeout - Time for the next timeout
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

void tcp_update_retrantimer(FAR struct tcp_conn_s *conn, int timeout)
{
  conn->timer = timeout;
  tcp_update_timer(conn);
}

/****************************************************************************
 * Name: tcp_update_keeptimer
 *
 * Description:
 *   Update the keeplive TCP timer for the provided TCP connection,
 *   The timeout is accurate
 *
 * Input Parameters:
 *   conn    - The TCP "connection" to poll for TX data
 *   timeout - Time for the next timeout
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_KEEPALIVE
void tcp_update_keeptimer(FAR struct tcp_conn_s *conn, int timeout)
{
  conn->keeptimer = timeout;
  tcp_update_timer(conn);
}
#endif

/****************************************************************************
 * Name: tcp_stop_timer
 *
 * Description:
 *   Stop TCP timer for the provided TCP connection
 *   When the connection is closed
 *
 * Input Parameters:
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   conn is not NULL.
 *
 ****************************************************************************/

void tcp_stop_timer(FAR struct tcp_conn_s *conn)
{
  work_cancel(LPWORK, &conn->work);
}

/****************************************************************************
 * Name: tcp_timer
 *
 * Description:
 *   Handle a TCP timer expiration for the provided TCP connection
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *   dev is not NULL.
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

void tcp_timer(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn)
{
  int hsec = tcp_get_timeout(conn);
  uint16_t result;
  uint8_t hdrlen;

  /* NOTE: It is important to decrease conn->timer at "hsec" pace,
   * not faster. Excessive (false) decrements of conn->timer are not allowed
   * here. Otherwise, it breaks TCP timings and leads to TCP spurious
   * retransmissions and other issues due to premature timeouts.
   */

  DEBUGASSERT(dev != NULL && conn != NULL && dev == conn->dev);

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

  if (conn->tcpstateflags == TCP_CLOSED)
    {
      /* Nothing to be done */

      return;
    }

  /* Check if the connection is in a state in which we simply wait
   * for the connection to time out. If so, we increase the
   * connection's timer and remove the connection if it times
   * out.
   */

  if (conn->tcpstateflags == TCP_TIME_WAIT ||
      conn->tcpstateflags == TCP_FIN_WAIT_2)
    {
      /* Check if the timer exceeds the timeout value */

      if (conn->timer <= hsec)
        {
          /* Set the timer to zero value */

          conn->timer         = 0;
          conn->tcpstateflags = TCP_CLOSED;

          /* Notify upper layers about the timeout */

          tcp_callback(dev, conn, TCP_TIMEDOUT);

          ninfo("TCP state: TCP_CLOSED\n");
        }
      else
        {
          /* No timeout. Just update the decremented timer */

          conn->timer -= hsec;
        }
    }
  else if (conn->tcpstateflags != TCP_CLOSED)
    {
      /* If the connection has outstanding data, we increase the connection's
       * timer and see if it has reached the RTO value in which case we
       * retransmit.
       */

      if (conn->tx_unacked > 0)
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
                  listener = tcp_findlistener(&conn->u, conn->lport,
                                              conn->domain);
#else
                  listener = tcp_findlistener(&conn->u, conn->lport);
#endif
                  if (listener != NULL)
                    {
                      /* We call tcp_callback() for the connection with
                       * TCP_TIMEDOUT to inform the listener that the
                       * connection has timed out.
                       */

                      tcp_callback(dev, listener, TCP_TIMEDOUT);
                    }

                  /* We also send a reset packet to the remote host. */

                  tcp_send(dev, conn, TCP_RST | TCP_ACK, hdrlen);

                  /* Finally, we must free this TCP connection structure */

                  conn->crefs = 0;
                  tcp_free(conn);
                  goto done;
                }

              /* Otherwise, check for a timeout on an established connection.
               * If the retry count is exceeded in this case, we should
               * close the connection.
               */

              else if (
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#  ifdef CONFIG_NET_SENDFILE
                  (!conn->sendfile && conn->expired > 0) ||
                  (conn->sendfile && conn->nrtx >= TCP_MAXRTX) ||
#  else
                  conn->expired > 0 ||
#  endif
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

                  tcp_callback(dev, conn, TCP_TIMEDOUT);

                  /* We also send a reset packet to the remote host. */

                  tcp_send(dev, conn, TCP_RST | TCP_ACK, hdrlen);
                  goto done;
                }

              /* Exponential backoff. */

              conn->timer = TCP_RTO << (conn->nrtx > 4 ? 4: conn->nrtx);
              conn->nrtx++;

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

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                    tcp_setsequence(conn->sndseq, conn->rexmit_seq);
#else
                    /* REVISIT for the buffered mode */
#endif
                    tcp_synack(dev, conn, TCP_ACK | TCP_SYN);
                    goto done;

                  case TCP_SYN_SENT:

                    /* In the SYN_SENT state, we retransmit out SYN. */

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                    tcp_setsequence(conn->sndseq, conn->rexmit_seq);
#else
                    /* REVISIT for the buffered mode */
#endif
                    tcp_synack(dev, conn, TCP_SYN);
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

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                    tcp_setsequence(conn->sndseq, conn->rexmit_seq);
#else
                    /* REVISIT for the buffered mode */
#endif
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
#ifdef CONFIG_NET_TCP_KEEPALIVE
          /* Is this an established connected with KeepAlive enabled? */

          if (conn->keepalive)
            {
              uint32_t saveseq;

              /* Yes... has the idle period elapsed with no data or ACK
               * received from the remote peer?
               */

              if (conn->keeptimer > hsec)
                {
                  /* Will not yet decrement to zero */

                  conn->keeptimer -= hsec;
                }
              else
                {
                  /* Yes.. Has the retry count expired? */

                  if (conn->keepretries >= conn->keepcnt)
                    {
                      /* Yes... stop the network monitor, closing the
                       * connection and all sockets associated with the
                       * connection.
                       */

                      tcp_stop_monitor(conn, TCP_ABORT);
                    }
                  else
                    {
                      unsigned int tcpiplen;

                      /* No.. we need to send another probe.
                       * Get the size of the IP and TCP header.
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

                      /* And send the probe.
                       * The packet we send must have these properties:
                       *
                       *   - TCP_ACK flag (only) is set.
                       *   - Sequence number is the sequence number of
                       *     previously ACKed data, i.e., the expected
                       *     sequence number minus one.
                       *
                       * tcp_send() will send the TCP sequence number as
                       * conn->sndseq.  Rather than creating a new
                       * interface, we spoof tcp_end() here:
                       */

                      saveseq = tcp_getsequence(conn->sndseq);
                      tcp_setsequence(conn->sndseq, saveseq - 1);

                      tcp_send(dev, conn, TCP_ACK, tcpiplen);

                      tcp_setsequence(conn->sndseq, saveseq);

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
                      /* Increment the un-ACKed sequence number */

                      conn->sndseq_max++;
#endif
                      /* Update for the next probe */

                      conn->keeptimer = conn->keepintvl;
                      conn->keepretries++;
                    }

                  goto done;
                }
            }
#endif

#ifdef CONFIG_NET_TCP_DELAYED_ACK
          /* Handle delayed acknowledgments.  Is there a segment with a
           * delayed acknowledgment?
           */

          if (conn->rx_unackseg > 0)
            {
              /* Increment the ACK delay. */

              conn->rx_acktimer += hsec;

              /* Per RFC 1122:  "...an ACK should not be excessively
               * delayed; in particular, the delay must be less than
               * 0.5 seconds..."
               */

              if (conn->rx_acktimer >= ACK_DELAY)
                {
                  /* Reset the delayed ACK state and send the ACK
                   * packet.
                   */

                  conn->rx_unackseg = 0;
                  conn->rx_acktimer = 0;
                  tcp_synack(dev, conn, TCP_ACK);
                  goto done;
                }
            }
#endif

          /* There was no need for a retransmission and there was no
           * need to probe the remote peer and there was no need to
           * send a delayed ACK.  We poll the application for new
           * outgoing data.
           */

          result = tcp_callback(dev, conn, TCP_POLL);
          tcp_appsend(dev, conn, result);
          goto done;
        }
    }

  /* Nothing to be done */

  dev->d_len = 0;

done:
  tcp_update_timer(conn);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
