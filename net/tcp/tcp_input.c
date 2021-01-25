/****************************************************************************
 * net/tcp/tcp_input.c
 * Handling incoming TCP input
 *
 *   Copyright (C) 2007-2014, 2017-2019, 2020 Gregory Nutt. All rights
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF ((FAR struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_input
 *
 * Description:
 *   Handle incoming TCP input
 *
 * Input Parameters:
 *   dev    - The device driver structure containing the received TCP packet.
 *   domain - IP domain (PF_INET or PF_INET6)
 *   iplen  - Length of the IP header (IPv4_HDRLEN or IPv6_HDRLEN).
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void tcp_input(FAR struct net_driver_s *dev, uint8_t domain,
                      unsigned int iplen)
{
  FAR struct tcp_hdr_s *tcp;
  FAR struct tcp_conn_s *conn = NULL;
  unsigned int tcpiplen;
  unsigned int hdrlen;
  uint16_t tmp16;
  uint16_t flags;
  uint16_t result;
  uint8_t  opt;
  int      len;
  int      i;

#ifdef CONFIG_NET_STATISTICS
  /* Bump up the count of TCP packets received */

  g_netstats.tcp.recv++;
#endif

  /* Get a pointer to the TCP header.  The TCP header lies just after the
   * the link layer header and the IP header.
   */

  tcp = (FAR struct tcp_hdr_s *)&dev->d_buf[iplen + NET_LL_HDRLEN(dev)];

  /* Get the size of the IP header and the TCP header.
   *
   * REVISIT:  TCP header is *not* a constant!  It can be larger if the
   * TCP header includes options.  The constant TCP_HDRLEN should be
   * replaced with the macro TCP_OPT_HDRLEN(n) which will calculate the
   * correct header length in all cases.
   */

  tcpiplen = iplen + TCP_HDRLEN;

  /* Get the size of the link layer header, the IP and TCP header */

  hdrlen = tcpiplen + NET_LL_HDRLEN(dev);

  /* Start of TCP input header processing code. */

  if (tcp_chksum(dev) != 0xffff)
    {
      /* Compute and check the TCP checksum. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.tcp.drop++;
      g_netstats.tcp.chkerr++;
#endif
      nwarn("WARNING: Bad TCP checksum\n");
      goto drop;
    }

  /* Demultiplex this segment. First check any active connections. */

  conn = tcp_active(dev, tcp);
  if (conn)
    {
      /* We found an active connection.. Check for the subsequent SYN
       * arriving in TCP_SYN_RCVD state after the SYNACK packet was
       * lost.  To avoid other issues,  reset any active connection
       * where a SYN arrives in a state != TCP_SYN_RCVD.
       */

      if ((conn->tcpstateflags & TCP_STATE_MASK) != TCP_SYN_RCVD &&
          (tcp->flags & TCP_CTL) == TCP_SYN)
        {
          nwarn("WARNING: SYN in TCP_SYN_RCVD\n");
          goto reset;
        }
      else
        {
          goto found;
        }
    }

  /* If we didn't find an active connection that expected the packet,
   * either (1) this packet is an old duplicate, or (2) this is a SYN packet
   * destined for a connection in LISTEN.  If the SYN flag isn't set,
   * it is an old packet and we send a RST.
   */

  if ((tcp->flags & TCP_CTL) == TCP_SYN)
    {
      /* This is a SYN packet for a connection.  Find the connection
       * listening on this port.
       */

      tmp16 = tcp->destport;
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      if (tcp_islistener(tmp16, domain))
#else
      if (tcp_islistener(tmp16))
#endif
        {
          /* We matched the incoming packet with a connection in LISTEN.
           * We now need to create a new connection and send a SYNACK in
           * response.
           */

          /* First allocate a new connection structure and see if there is
           * any user application to accept it.
           */

          conn = tcp_alloc_accept(dev, tcp);
          if (conn)
            {
              /* The connection structure was successfully allocated and has
               * been initialized in the TCP_SYN_RECVD state.  The expected
               * sequence of events is then the rest of the 3-way handshake:
               *
               *  1. We just received a TCP SYN packet from a remote host.
               *  2. We will send the SYN-ACK response below (perhaps
               *     repeatedly in the event of a timeout)
               *  3. Then we expect to receive an ACK from the remote host
               *     indicated the TCP socket connection is ESTABLISHED.
               *
               * Possible failure:
               *
               *  1. The ACK is never received.  This will be handled by
               *     a timeout managed by tcp_timer().
               *  2. The listener "unlistens()".  This will be handled by
               *     the failure of tcp_accept_connection() when the ACK is
               *     received.
               */

              conn->crefs = 1;
            }

          if (!conn)
            {
              /* Either (1) all available connections are in use, or (2)
               * there is no application in place to accept the connection.
               * We drop packet and hope that the remote end will retransmit
               * the packet at a time when we have more spare connections
               * or someone waiting to accept the connection.
               */

#ifdef CONFIG_NET_STATISTICS
              g_netstats.tcp.syndrop++;
#endif
              nerr("ERROR: No free TCP connections\n");
              goto drop;
            }

          net_incr32(conn->rcvseq, 1);

          /* Parse the TCP MSS option, if present. */

          if ((tcp->tcpoffset & 0xf0) > 0x50)
            {
              for (i = 0; i < ((tcp->tcpoffset >> 4) - 5) << 2 ; )
                {
                  opt = dev->d_buf[hdrlen + i];
                  if (opt == TCP_OPT_END)
                    {
                      /* End of options. */

                      break;
                    }
                  else if (opt == TCP_OPT_NOOP)
                    {
                      /* NOP option. */

                      ++i;
                    }
                  else if (opt == TCP_OPT_MSS &&
                          dev->d_buf[hdrlen + 1 + i] == TCP_OPT_MSS_LEN)
                    {
                      uint16_t tcp_mss = TCP_MSS(dev, iplen);

                      /* An MSS option with the right option length. */

                      tmp16 = ((uint16_t)dev->d_buf[hdrlen + 2 + i] << 8) |
                               (uint16_t)dev->d_buf[hdrlen + 3 + i];
                      conn->mss = tmp16 > tcp_mss ? tcp_mss : tmp16;

                      /* And we are done processing options. */

                      break;
                    }
                  else
                    {
                      /* All other options have a length field, so that we
                       * easily can skip past them.
                       */

                      if (dev->d_buf[hdrlen + 1 + i] == 0)
                        {
                          /* If the length field is zero, the options are
                           * malformed and we don't process them further.
                           */

                          break;
                        }

                      i += dev->d_buf[hdrlen + 1 + i];
                    }
                }
            }

          /* Our response will be a SYNACK. */

          tcp_synack(dev, conn, TCP_ACK | TCP_SYN);
          return;
        }
    }

  nwarn("WARNING: SYN with no listener (or old packet) .. reset\n");

  /* This is (1) an old duplicate packet or (2) a SYN packet but with
   * no matching listener found.  Send RST packet in either case.
   */

reset:

  /* We do not send resets in response to resets. */

  if ((tcp->flags & TCP_RST) != 0)
    {
      goto drop;
    }

#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.synrst++;
#endif
  tcp_reset(dev);
  return;

found:

  /* Update the connection's window size */

  conn->snd_wnd = ((uint16_t)tcp->wnd[0] << 8) + (uint16_t)tcp->wnd[1];

  flags = 0;

  /* We do a very naive form of TCP reset processing; we just accept
   * any RST and kill our connection. We should in fact check if the
   * sequence number of this reset is within our advertised window
   * before we accept the reset.
   */

  if ((tcp->flags & TCP_RST) != 0)
    {
      FAR struct tcp_conn_s *listener = NULL;

      /* An RST received during the 3-way connection handshake requires
       * little more clean-up.
       */

      if ((conn->tcpstateflags & TCP_STATE_MASK) == TCP_SYN_RCVD)
        {
          conn->tcpstateflags = TCP_CLOSED;
          nwarn("WARNING: RESET in TCP_SYN_RCVD\n");

          /* Notify the listener for the connection of the reset event */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          listener = tcp_findlistener(conn->lport, domain);
#else
          listener = tcp_findlistener(conn->lport);
#endif

          /* We must free this TCP connection structure; this connection
           * will never be established.  There should only be one reference
           * on this connection when we allocated for the connection.
           */

          DEBUGASSERT(conn->crefs == 1);
          conn->crefs = 0;
          tcp_free(conn);
        }
      else
        {
          conn->tcpstateflags = TCP_CLOSED;
          nwarn("WARNING: RESET TCP state: TCP_CLOSED\n");

          /* Notify this connection of the reset event */

          listener = conn;
        }

      /* Perform the TCP_ABORT callback and drop the packet */

      if (listener != NULL)
        {
          tcp_callback(dev, listener, TCP_ABORT);
        }

      goto drop;
    }

  /* Calculated the length of the data, if the application has sent
   * any data to us.
   */

  len = (tcp->tcpoffset >> 4) << 2;

  /* d_len will contain the length of the actual TCP data. This is
   * calculated by subtracting the length of the TCP header (in
   * len) and the length of the IP header.
   */

  dev->d_len -= (len + iplen);

#ifdef CONFIG_NET_TCP_KEEPALIVE
  /* Check for a to KeepAlive probes.  These packets have these properties:
   *
   *   - TCP_ACK flag is set.  SYN/FIN/RST never appear in a Keepalive probe.
   *   - Sequence number is the sequence number of previously ACKed data,
   *     i.e., the expected sequence number minus one.
   *   - The data payload is one or two bytes.
   *
   * We would expect a KeepAlive only in the ESTABLISHED state and only after
   * some time has elapsed with no network activity.  If there is un-ACKed
   * data, then we will let the normal TCP re-transmission logic handle that
   * case.
   */

  if ((tcp->flags & TCP_ACK) != 0 &&
      (tcp->flags & (TCP_SYN | TCP_FIN | TCP_RST)) == 0 &&
      (conn->tcpstateflags & TCP_STATE_MASK) == TCP_ESTABLISHED &&
      (dev->d_len == 0 || dev->d_len == 1) &&
      conn->tx_unacked <= 0)
    {
      uint32_t ackseq;
      uint32_t rcvseq;

      /* Get the sequence number of that has just been acknowledged by this
       * incoming packet.
       */

      ackseq = tcp_getsequence(tcp->seqno);
      rcvseq = tcp_getsequence(conn->rcvseq);

      if (ackseq < rcvseq)
        {
          /* Send a "normal" acknowledgment of the KeepAlive probe */

          tcp_send(dev, conn, TCP_ACK, tcpiplen);
          return;
        }
    }
#endif

  /* Check if the sequence number of the incoming packet is what we are
   * expecting next.  If not, we send out an ACK with the correct numbers
   * in, unless we are in the SYN_RCVD state and receive a SYN, in which
   * case we should retransmit our SYNACK (which is done further down).
   */

  if (!((((conn->tcpstateflags & TCP_STATE_MASK) == TCP_SYN_SENT) &&
        ((tcp->flags & TCP_CTL) == (TCP_SYN | TCP_ACK))) ||
        (((conn->tcpstateflags & TCP_STATE_MASK) == TCP_SYN_RCVD) &&
        ((tcp->flags & TCP_CTL) == TCP_SYN))))
    {
      if ((dev->d_len > 0 || ((tcp->flags & (TCP_SYN | TCP_FIN)) != 0)) &&
          memcmp(tcp->seqno, conn->rcvseq, 4) != 0)
        {
          tcp_send(dev, conn, TCP_ACK, tcpiplen);
          return;
        }
    }

  /* Check if the incoming segment acknowledges any outstanding data. If so,
   * we update the sequence number, reset the length of the outstanding
   * data, calculate RTT estimations, and reset the retransmission timer.
   */

  if ((tcp->flags & TCP_ACK) != 0 && conn->tx_unacked > 0)
    {
      uint32_t unackseq;
      uint32_t ackseq;

      /* The next sequence number is equal to the current sequence
       * number (sndseq) plus the size of the outstanding, unacknowledged
       * data (tx_unacked).
       */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      unackseq = conn->sndseq_max;
#else
      unackseq = tcp_addsequence(conn->sndseq, conn->tx_unacked);
#endif

      /* Get the sequence number of that has just been acknowledged by this
       * incoming packet.
       */

      ackseq = tcp_getsequence(tcp->ackno);

      /* Check how many of the outstanding bytes have been acknowledged. For
       * most send operations, this should always be true.  However,
       * the send() API sends data ahead when it can without waiting for
       * the ACK.  In this case, the 'ackseq' could be less than then the
       * new sequence number.
       */

      if (ackseq <= unackseq)
        {
          /* Calculate the new number of outstanding, unacknowledged bytes */

          conn->tx_unacked = unackseq - ackseq;
        }
      else
        {
          /* What would it mean if ackseq > unackseq?  The peer has ACKed
           * more bytes than we think we have sent?  Someone has lost it.
           * Complain and reset the number of outstanding, unacknowledged
           * bytes
           */

          if ((conn->tcpstateflags & TCP_STATE_MASK) == TCP_ESTABLISHED)
            {
              nwarn("WARNING: ackseq > unackseq\n");
              nwarn("sndseq=%" PRIu32 " tx_unacked=%" PRIu32
                    " unackseq=%" PRIu32 " ackseq=%" PRIu32 "\n",
                    tcp_getsequence(conn->sndseq),
                    (uint32_t)conn->tx_unacked,
                    unackseq, ackseq);

              conn->tx_unacked = 0;
            }
        }

      /* Update sequence number to the unacknowledge sequence number.  If
       * there is still outstanding, unacknowledged data, then this will
       * be beyond ackseq.
       */

      ninfo("sndseq: %08" PRIx32 "->%08" PRIx32
            " unackseq: %08" PRIx32 " new tx_unacked: %" PRId32 "\n",
            tcp_getsequence(conn->sndseq), ackseq, unackseq,
            (uint32_t)conn->tx_unacked);
      tcp_setsequence(conn->sndseq, ackseq);

      /* Do RTT estimation, unless we have done retransmissions. */

      if (conn->nrtx == 0)
        {
          signed char m;
          m = conn->rto - conn->timer;

          /* This is taken directly from VJs original code in his paper */

          m = m - (conn->sa >> 3);
          conn->sa += m;
          if (m < 0)
            {
              m = -m;
            }

          m = m - (conn->sv >> 2);
          conn->sv += m;
          conn->rto = (conn->sa >> 3) + conn->sv;
        }

      /* Set the acknowledged flag. */

      flags |= TCP_ACKDATA;

      /* Reset the retransmission timer. */

      conn->timer = conn->rto;
    }

  /* Do different things depending on in what state the connection is. */

  switch (conn->tcpstateflags & TCP_STATE_MASK)
    {
      /* CLOSED and LISTEN are not handled here. CLOSE_WAIT is not
       * implemented, since we force the application to close when the
       * peer sends a FIN (hence the application goes directly from
       * ESTABLISHED to LAST_ACK).
       */

      case TCP_SYN_RCVD:
        /* In SYN_RCVD we have sent out a SYNACK in response to a SYN, and
         * we are waiting for an ACK that acknowledges the data we sent
         * out the last time. Therefore, we want to have the TCP_ACKDATA
         * flag set. If so, we enter the ESTABLISHED state.
         */

        if ((flags & TCP_ACKDATA) != 0)
          {
            /* The three way handshake is complete and the TCP connection
             * is now in the ESTABLISHED state.
             */

            conn->tcpstateflags = TCP_ESTABLISHED;

            /* Wake up any listener waiting for a connection on this port */

            if (tcp_accept_connection(dev, conn, tcp->destport) != OK)
              {
                /* No more listener for current port.  We can free conn here
                 * because it has not been shared with upper layers yet as
                 * handshake is not complete
                 */

                nwarn("WARNING: Listen canceled while waiting for ACK on "
                      "port %d\n",
                      ntohs(tcp->destport));

                /* Free the connection structure */

                conn->crefs = 0;
                tcp_free(conn);
                conn = NULL;

                /* And send a reset packet to the remote host. */

                goto reset;
              }

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
            conn->isn           = tcp_getsequence(tcp->ackno);
            tcp_setsequence(conn->sndseq, conn->isn);
            conn->sent          = 0;
            conn->sndseq_max    = 0;
#endif
            conn->tx_unacked    = 0;
            flags               = TCP_CONNECTED;
            ninfo("TCP state: TCP_ESTABLISHED\n");

            if (dev->d_len > 0)
              {
                flags          |= TCP_NEWDATA;
                net_incr32(conn->rcvseq, dev->d_len);
              }

            dev->d_sndlen       = 0;
            result              = tcp_callback(dev, conn, flags);
            tcp_appsend(dev, conn, result);
            return;
          }

        /* We need to retransmit the SYNACK */

        if ((tcp->flags & TCP_CTL) == TCP_SYN)
          {
            tcp_synack(dev, conn, TCP_ACK | TCP_SYN);
            return;
          }

        goto drop;

      case TCP_SYN_SENT:
        /* In SYN_SENT, we wait for a SYNACK that is sent in response to
         * our SYN. The rcvseq is set to sequence number in the SYNACK
         * plus one, and we send an ACK. We move into the ESTABLISHED
         * state.
         */

        if ((flags & TCP_ACKDATA) != 0 &&
            (tcp->flags & TCP_CTL) == (TCP_SYN | TCP_ACK))
          {
            /* Parse the TCP MSS option, if present. */

            if ((tcp->tcpoffset & 0xf0) > 0x50)
              {
                for (i = 0; i < ((tcp->tcpoffset >> 4) - 5) << 2 ; )
                  {
                    opt = dev->d_buf[hdrlen + i];
                    if (opt == TCP_OPT_END)
                      {
                        /* End of options. */

                        break;
                      }
                    else if (opt == TCP_OPT_NOOP)
                      {
                        /* NOP option. */

                        ++i;
                      }
                    else if (opt == TCP_OPT_MSS &&
                              dev->d_buf[hdrlen + 1 + i] == TCP_OPT_MSS_LEN)
                      {
                        uint16_t tcp_mss = TCP_MSS(dev, iplen);

                        /* An MSS option with the right option length. */

                        tmp16 =
                          (dev->d_buf[hdrlen + 2 + i] << 8) |
                          dev->d_buf[hdrlen + 3 + i];
                        conn->mss = tmp16 > tcp_mss ? tcp_mss : tmp16;

                        /* And we are done processing options. */

                        break;
                      }
                    else
                      {
                        /* All other options have a length field, so that we
                         * easily can skip past them.
                         */

                        if (dev->d_buf[hdrlen + 1 + i] == 0)
                          {
                            /* If the length field is zero, the options are
                             * malformed and we don't process them further.
                             */

                            break;
                          }

                        i += dev->d_buf[hdrlen + 1 + i];
                      }
                  }
              }

            conn->tcpstateflags = TCP_ESTABLISHED;
            memcpy(conn->rcvseq, tcp->seqno, 4);

            net_incr32(conn->rcvseq, 1);
            conn->tx_unacked    = 0;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
            conn->isn           = tcp_getsequence(tcp->ackno);
            tcp_setsequence(conn->sndseq, conn->isn);
#endif
            dev->d_len          = 0;
            dev->d_sndlen       = 0;

            ninfo("TCP state: TCP_ESTABLISHED\n");
            result = tcp_callback(dev, conn, TCP_CONNECTED | TCP_NEWDATA);
            tcp_appsend(dev, conn, result);
            return;
          }

        /* Inform the application that the connection failed */

        tcp_callback(dev, conn, TCP_ABORT);

        /* The connection is closed after we send the RST */

        conn->tcpstateflags = TCP_CLOSED;
        ninfo("Connection failed - TCP state: TCP_CLOSED\n");

        /* We do not send resets in response to resets. */

        if ((tcp->flags & TCP_RST) != 0)
          {
            goto drop;
          }

        tcp_reset(dev);
        return;

      case TCP_ESTABLISHED:
        /* In the ESTABLISHED state, we call upon the application to feed
         * data into the d_buf.  If the TCP_ACKDATA flag is set, the
         * application should put new data into the buffer, otherwise we are
         * retransmitting an old segment, and the application should put that
         * data into the buffer.
         *
         * If the incoming packet is a FIN, we should close the connection on
         * this side as well, and we send out a FIN and enter the LAST_ACK
         * state.  We require that there is no outstanding data; otherwise
         * the sequence numbers will be screwed up.
         */

        if ((tcp->flags & TCP_FIN) != 0 &&
            (conn->tcpstateflags & TCP_STOPPED) == 0)
          {
            /* Needs to be investigated further.
             * Windows often sends FIN packets together with the last ACK for
             * the received data. So the socket layer has to get this ACK
             * even if the connection is going to be closed.
             */

#if 0
            if (conn->tx_unacked > 0)
              {
                goto drop;
              }
#endif

            /* Update the sequence number and indicate that the connection
             * has been closed.
             */

            net_incr32(conn->rcvseq, dev->d_len + 1);
            flags |= TCP_CLOSE;

            if (dev->d_len > 0)
              {
                flags |= TCP_NEWDATA;
              }

            tcp_callback(dev, conn, flags);

            conn->tcpstateflags = TCP_LAST_ACK;
            conn->tx_unacked    = 1;
            conn->nrtx          = 0;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
            conn->sndseq_max    = tcp_getsequence(conn->sndseq) + 1;
#endif
            ninfo("TCP state: TCP_LAST_ACK\n");

            tcp_send(dev, conn, TCP_FIN | TCP_ACK, tcpiplen);
            return;
          }

#ifdef CONFIG_NET_TCPURGDATA
        /* Check the URG flag.  If this is set, the segment carries urgent
         * data that we must pass to the application.
         */

        if ((tcp->flags & TCP_URG) != 0)
          {
            dev->d_urglen = (tcp->urgp[0] << 8) | tcp->urgp[1];
            if (dev->d_urglen > dev->d_len)
              {
                /* There is more urgent data in the next segment to come. */

                dev->d_urglen = dev->d_len;
              }

             /* The d_len field contains the length of the incoming data.
              * d_urgdata points to the "urgent" data at the beginning of
              * the payload; d_appdata field points to the any "normal" data
              * that may follow the urgent data.
              *
              * NOTE: If the urgent data continues in the next packet, then
              * d_len will be zero and d_appdata will point past the end of
              * the payload (which is OK).
              */

            net_incr32(conn->rcvseq, dev->d_urglen);
            dev->d_len     -= dev->d_urglen;
            dev->d_urgdata  = dev->d_appdata;
            dev->d_appdata += dev->d_urglen;
          }
        else
          {
            /* No urgent data */

            dev->d_urglen   = 0;
          }

#else /* CONFIG_NET_TCPURGDATA */
        /* Check the URG flag.  If this is set, We must gracefully ignore
         * and discard the urgent data.
         */

        if ((tcp->flags & TCP_URG) != 0)
          {
            uint16_t urglen = (tcp->urgp[0] << 8) | tcp->urgp[1];
            if (urglen > dev->d_len)
              {
                /* There is more urgent data in the next segment to come. */

                urglen = dev->d_len;
              }

             /* The d_len field contains the length of the incoming data;
              * The d_appdata field points to the any "normal" data that
              * may follow the urgent data.
              *
              * NOTE: If the urgent data continues in the next packet, then
              * d_len will be zero and d_appdata will point past the end of
              * the payload (which is OK).
              */

            net_incr32(conn->rcvseq, urglen);
            dev->d_len     -= urglen;
            dev->d_appdata += urglen;
          }
#endif /* CONFIG_NET_TCPURGDATA */

#ifdef CONFIG_NET_TCP_KEEPALIVE
        /* If the established socket receives an ACK or any kind of data
         * from the remote peer (whether we accept it or not), then reset
         * the keep alive timer.
         */

        if (conn->keepalive &&
            (dev->d_len > 0 || (tcp->flags & TCP_ACK) != 0))
          {
            /* Reset the last known "alive" time.
             *
             * REVISIT:  At this level, we don't actually know if keep-
             * alive is enabled for this connection.
             */

            conn->keeptime    = clock_systime_ticks();
            conn->keepretries = 0;
          }
#endif

        /* If d_len > 0 we have TCP data in the packet, and we flag this
         * by setting the TCP_NEWDATA flag. If the application has stopped
         * the data flow using TCP_STOPPED, we must not accept any data
         * packets from the remote host.
         */

        if (dev->d_len > 0 && (conn->tcpstateflags & TCP_STOPPED) == 0)
          {
            flags |= TCP_NEWDATA;
          }

        /* If this packet constitutes an ACK for outstanding data (flagged
         * by the TCP_ACKDATA flag), we should call the application since it
         * might want to send more data.  If the incoming packet had data
         * from the peer (as flagged by the TCP_NEWDATA flag), the
         * application must also be notified.
         *
         * When the application is called, the d_len field
         * contains the length of the incoming data.  The application can
         * access the incoming data through the global pointer
         * d_appdata, which usually points hdrlen bytes into the d_buf
         * array.
         *
         * If the application wishes to send any data, this data should be
         * put into the d_appdata and the length of the data should be
         * put into d_len.  If the application don't have any data to
         * send, d_len must be set to 0.
         */

        if ((flags & (TCP_NEWDATA | TCP_ACKDATA)) != 0)
          {
            /* Clear sndlen and remember the size in d_len.  The application
             * may modify d_len and we will need this value later when we
             * update the sequence number.
             */

            dev->d_sndlen = 0;
            len           = dev->d_len;

            /* Provide the packet to the application */

            result = tcp_callback(dev, conn, flags);

            /* If the application successfully handled the incoming data,
             * then TCP_SNDACK will be set in the result.  In this case,
             * we need to update the sequence number.  The ACK will be
             * send by tcp_appsend().
             */

            if ((result & TCP_SNDACK) != 0)
              {
                /* Update the sequence number using the saved length */

                net_incr32(conn->rcvseq, len);
              }

            /* Send the response, ACKing the data or not, as appropriate */

            tcp_appsend(dev, conn, result);
            return;
          }

        goto drop;

      case TCP_LAST_ACK:
        /* We can close this connection if the peer has acknowledged our
         * FIN. This is indicated by the TCP_ACKDATA flag.
         */

        if ((flags & TCP_ACKDATA) != 0)
          {
            conn->tcpstateflags = TCP_CLOSED;
            ninfo("TCP_LAST_ACK TCP state: TCP_CLOSED\n");

            tcp_callback(dev, conn, TCP_CLOSE);
          }
        break;

      case TCP_FIN_WAIT_1:
        /* The application has closed the connection, but the remote host
         * hasn't closed its end yet.  Thus we stay in the FIN_WAIT_1 state
         * until we receive a FIN from the remote.
         */

        if (dev->d_len > 0)
          {
            net_incr32(conn->rcvseq, dev->d_len);
          }

        if ((tcp->flags & TCP_FIN) != 0)
          {
            if ((flags & TCP_ACKDATA) != 0 && conn->tx_unacked == 0)
              {
                conn->tcpstateflags = TCP_TIME_WAIT;
                conn->timer         = 0;
                ninfo("TCP state: TCP_TIME_WAIT\n");
              }
            else
              {
                conn->tcpstateflags = TCP_CLOSING;
                ninfo("TCP state: TCP_CLOSING\n");
              }

            net_incr32(conn->rcvseq, 1);
            tcp_callback(dev, conn, TCP_CLOSE);
            tcp_send(dev, conn, TCP_ACK, tcpiplen);
            return;
          }
        else if ((flags & TCP_ACKDATA) != 0 && conn->tx_unacked == 0)
          {
            conn->tcpstateflags = TCP_FIN_WAIT_2;
            ninfo("TCP state: TCP_FIN_WAIT_2\n");
            goto drop;
          }

        if (dev->d_len > 0)
          {
            tcp_send(dev, conn, TCP_ACK, tcpiplen);
            return;
          }

        goto drop;

      case TCP_FIN_WAIT_2:
        if (dev->d_len > 0)
          {
            net_incr32(conn->rcvseq, dev->d_len);
          }

        if ((tcp->flags & TCP_FIN) != 0)
          {
            conn->tcpstateflags = TCP_TIME_WAIT;
            conn->timer         = 0;
            ninfo("TCP state: TCP_TIME_WAIT\n");

            net_incr32(conn->rcvseq, 1);
            tcp_callback(dev, conn, TCP_CLOSE);
            tcp_send(dev, conn, TCP_ACK, tcpiplen);
            return;
          }

        if (dev->d_len > 0)
          {
            tcp_send(dev, conn, TCP_ACK, tcpiplen);
            return;
          }

        goto drop;

      case TCP_TIME_WAIT:
        tcp_send(dev, conn, TCP_ACK, tcpiplen);
        return;

      case TCP_CLOSING:
        if ((flags & TCP_ACKDATA) != 0)
          {
            conn->tcpstateflags = TCP_TIME_WAIT;
            conn->timer        = 0;
            ninfo("TCP state: TCP_TIME_WAIT\n");
          }

      default:
        break;
    }

drop:
  dev->d_len = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_ipv4_input
 *
 * Description:
 *   Handle incoming TCP input with IPv4 header
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received TCP packet.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the Ethernet driver with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void tcp_ipv4_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  uint16_t iphdrlen;

  /* Configure to receive an TCP IPv4 packet */

  tcp_ipv4_select(dev);

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* Then process in the TCP IPv4 input */

  tcp_input(dev, PF_INET, iphdrlen);
}
#endif

/****************************************************************************
 * Name: tcp_ipv6_input
 *
 * Description:
 *   Handle incoming TCP input with IPv4 header
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received TCP packet.
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the Ethernet driver with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void tcp_ipv6_input(FAR struct net_driver_s *dev, unsigned int iplen)
{
  /* Configure to receive an TCP IPv6 packet */

  tcp_ipv6_select(dev);

  /* Then process in the TCP IPv6 input */

  tcp_input(dev, PF_INET6, iplen);
}
#endif

#endif /* CONFIG_NET  && CONFIG_NET_TCP */
