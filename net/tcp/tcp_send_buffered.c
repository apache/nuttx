/****************************************************************************
 * net/tcp/tcp_send_buffered.c
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

#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && \
    defined(CONFIG_NET_TCP_WRITE_BUFFERS)

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_NET_TCP_WRBUFFER_DEBUG)
/* Force debug output (from this file only) */

#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <sys/types.h>
#include <sys/socket.h>

#include <inttypes.h>
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
#include <nuttx/net/tcp.h>
#include <nuttx/net/net.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "utils/utils.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

/* Debug */

#ifdef CONFIG_NET_TCP_WRBUFFER_DUMP
#  define BUF_DUMP(msg,buf,len) lib_dumpbuffer(msg,buf,len)
#else
#  define BUF_DUMP(msg,buf,len)
#  undef  TCP_WBDUMP
#  define TCP_WBDUMP(msg,wrb,len,offset)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_insert_segment
 *
 * Description:
 *   Insert a new segment in a write buffer queue, keep the segment queue in
 *   ascending order of sequence number.
 *
 * Input Parameters:
 *   wrb   The segment to be inserted
 *   q     The write buffer queue in which to insert the segment
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void psock_insert_segment(FAR struct tcp_wrbuffer_s *wrb,
                                 FAR sq_queue_t *q)
{
  FAR sq_entry_t *entry = (FAR sq_entry_t *)wrb;
  FAR sq_entry_t *insert = NULL;

  FAR sq_entry_t *itr;
  for (itr = sq_peek(q); itr; itr = sq_next(itr))
    {
      FAR struct tcp_wrbuffer_s *wrb0 = (FAR struct tcp_wrbuffer_s *)itr;
      if (TCP_WBSEQNO(wrb0) < TCP_WBSEQNO(wrb))
        {
          insert = itr;
        }
      else
        {
          break;
        }
    }

  if (insert)
    {
      sq_addafter(insert, entry, q);
    }
  else
    {
      sq_addfirst(entry, q);
    }
}

/****************************************************************************
 * Name: psock_writebuffer_notify
 *
 * Description:
 *   The TCP connection has been lost.  Free all write buffers.
 *
 * Input Parameters:
 *   psock    The socket structure
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_NOTIFIER
static void psock_writebuffer_notify(FAR struct tcp_conn_s *conn)
{
  /* Check if all write buffers have been sent and ACKed */

  if (sq_empty(&conn->write_q) && sq_empty(&conn->unacked_q))
    {
      /* Notify any waiters that the write buffers have been drained. */

      tcp_writebuffer_signal(conn);
    }
}
#else
#  define psock_writebuffer_notify(conn)
#endif

static void retransmit_segment(FAR struct tcp_conn_s *conn,
                               FAR struct tcp_wrbuffer_s *wrb)
{
  uint16_t sent;

  /* Reset the number of bytes sent sent from the write buffer */

  sent = TCP_WBSENT(wrb);
  if (conn->tx_unacked > sent)
    {
      conn->tx_unacked -= sent;
    }
  else
    {
      conn->tx_unacked = 0;
    }

  if (conn->sent > sent)
    {
      conn->sent -= sent;
    }
  else
    {
      conn->sent = 0;
    }

  TCP_WBSENT(wrb) = 0;
  ninfo("REXMIT: wrb=%p sent=%u, "
        "conn tx_unacked=%" PRId32 " sent=%" PRId32 "\n",
        wrb, TCP_WBSENT(wrb), conn->tx_unacked, conn->sent);

  /* Free any write buffers that have exceed the retry count */

  if (++TCP_WBNRTX(wrb) >= TCP_MAXRTX)
    {
      nwarn("WARNING: Expiring wrb=%p nrtx=%u\n",
            wrb, TCP_WBNRTX(wrb));

      /* Return the write buffer to the free list */

      tcp_wrbuffer_release(wrb);

      /* Notify any waiters if the write buffers have been
       * drained.
       */

      psock_writebuffer_notify(conn);

      /* NOTE expired is different from un-ACKed, it is designed
       * to represent the number of segments that have been sent,
       * retransmitted, and un-ACKed, if expired is not zero, the
       * connection will be closed.
       *
       * field expired can only be updated at TCP_ESTABLISHED
       * state
       */

      conn->expired++;
    }
  else
    {
      /* Insert the write buffer into the write_q (in sequence
       * number order).  The retransmission will occur below
       * when the write buffer with the lowest sequence number
       * is pulled from the write_q again.
       */

      ninfo("REXMIT: Moving wrb=%p nrtx=%u\n",
            wrb, TCP_WBNRTX(wrb));

      psock_insert_segment(wrb, &conn->write_q);
    }
}

/****************************************************************************
 * Name: psock_lost_connection
 *
 * Description:
 *   The TCP connection has been lost.  Free all write buffers.
 *
 * Input Parameters:
 *   psock    The socket structure
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void psock_lost_connection(FAR struct tcp_conn_s *conn,
                                         bool abort)
{
  FAR sq_entry_t *entry;
  FAR sq_entry_t *next;

  /* Do not allow any further callbacks */

  if (conn->sndcb != NULL)
    {
      conn->sndcb->flags = 0;
      conn->sndcb->event = NULL;
    }

  if (conn != NULL)
    {
      /* Free all queued write buffers */

      for (entry = sq_peek(&conn->unacked_q); entry; entry = next)
        {
          next = sq_next(entry);
          tcp_wrbuffer_release((FAR struct tcp_wrbuffer_s *)entry);
        }

      for (entry = sq_peek(&conn->write_q); entry; entry = next)
        {
          next = sq_next(entry);
          tcp_wrbuffer_release((FAR struct tcp_wrbuffer_s *)entry);
        }

#if CONFIG_NET_SEND_BUFSIZE > 0
      /* Notify the send buffer available */

      tcp_sendbuffer_notify(conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */

      /* Reset write buffering variables */

      sq_init(&conn->unacked_q);
      sq_init(&conn->write_q);

      /* Notify any waiters if the write buffers have been drained. */

      psock_writebuffer_notify(conn);

      conn->sent       = 0;
      conn->sndseq_max = 0;

      /* Force abort the connection. */

      if (abort)
        {
          conn->tx_unacked = 0;
          conn->tcpstateflags = TCP_CLOSED;
        }
    }
}

/****************************************************************************
 * Name: send_ipselect
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
static inline void send_ipselect(FAR struct net_driver_s *dev,
                                 FAR struct tcp_conn_s *conn)
{
  /* Which domain the socket support */

  if (conn->domain == PF_INET)
    {
      /* Select the IPv4 domain */

      tcp_ipv4_select(dev);
    }
  else /* if (conn->domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      tcp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: parse_sack
 *
 * Description:
 *   Parse sack from incoming TCP options
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *   tcp    - Header of tcp structure
 *   segs   - Segments edge of sacks
 *
 * Returned Value:
 *   Number of sacks
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_SELECTIVE_ACK
static int parse_sack(FAR struct tcp_conn_s *conn, FAR struct tcp_hdr_s *tcp,
                      FAR struct tcp_ofoseg_s *segs)
{
  FAR struct tcp_sack_s *sacks;
  int nsack = 0;
  uint8_t opt;
  int i;

  /* Get the size of the link layer header,
   * the IP and TCP header
   */

  for (i = 0; i < ((tcp->tcpoffset >> 4) - 5) << 2 ; )
    {
      opt = *(tcp->optdata + i);
      if (opt == TCP_OPT_END)
        {
          /* End of options. */

          break;
        }
      else if (opt == TCP_OPT_NOOP)
        {
          /* NOP option. */

          ++i;
          continue;
        }
      else if (opt == TCP_OPT_SACK)
        {
          nsack = (*(tcp->optdata + 1 + i) -
                   TCP_OPT_SACK_PERM_LEN) /
                   (sizeof(uint32_t) * 2);
          sacks = (FAR struct tcp_sack_s *)
                  (tcp->optdata + i +
                   TCP_OPT_SACK_PERM_LEN);

          for (i = 0; i < nsack; i++)
            {
              segs[i].left = tcp_getsequence((uint8_t *)&sacks[i].left);
              segs[i].right = tcp_getsequence((uint8_t *)&sacks[i].right);
            }

          tcp_reorder_ofosegs(nsack, segs);

          break;
        }
      else
        {
          /* All other options have a length field,
           * so that we easily can skip past them.
           */

          if (*(tcp->optdata + 1 + i) == 0)
            {
              /* If the length field is zero,
               * the options are malformed and
               * we don't process them further.
               */

              break;
            }
        }

      i += *(tcp->optdata + 1 + i);
    }

  return nsack;
}
#endif /* CONFIG_NET_TCP_SELECTIVE_ACK */

/****************************************************************************
 * Name: psock_send_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct tcp_conn_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint16_t flags)
{
  FAR struct tcp_conn_s *conn = pvpriv;
#ifdef CONFIG_NET_TCP_SELECTIVE_ACK
  struct tcp_ofoseg_s ofosegs[TCP_SACK_RANGES_MAX];
  uint8_t nsacks = 0;
#endif
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
  uint32_t rexmitno = 0;
#endif

  /* Get the TCP connection pointer reliably from
   * the corresponding TCP socket.
   */

  DEBUGASSERT(conn != NULL);

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the one that we are bound to.
   */

  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      return flags;
    }

  ninfo("flags: %04x\n", flags);

  /* The TCP_ACKDATA, TCP_REXMIT and TCP_DISCONN_EVENTS flags are expected to
   * appear here strictly one at a time, except for the FIN + ACK case.
   */

  DEBUGASSERT((flags & TCP_ACKDATA) == 0 ||
              (flags & TCP_REXMIT) == 0);
  DEBUGASSERT((flags & TCP_DISCONN_EVENTS) == 0 ||
              (flags & TCP_REXMIT) == 0);

  /* If this packet contains an acknowledgment, then update the count of
   * acknowledged bytes.
   */

  if ((flags & TCP_ACKDATA) != 0)
    {
      FAR struct tcp_wrbuffer_s *wrb;
      FAR struct tcp_hdr_s *tcp;
      FAR sq_entry_t *entry;
      FAR sq_entry_t *next;
      uint32_t ackno;

      /* Get the offset address of the TCP header */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          tcp = TCPIPv4BUF;
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          tcp = TCPIPv6BUF;
        }
#endif /* CONFIG_NET_IPv6 */

      /* Get the ACK number from the TCP header */

      ackno = tcp_getsequence(tcp->ackno);
      ninfo("ACK: ackno=%" PRIu32 " flags=%04x\n", ackno, flags);

      /* Look at every write buffer in the unacked_q.  The unacked_q
       * holds write buffers that have been entirely sent, but which
       * have not yet been ACKed.
       */

      for (entry = sq_peek(&conn->unacked_q); entry; entry = next)
        {
          uint32_t lastseq;

          /* Check of some or all of this write buffer has been ACKed. */

          next = sq_next(entry);
          wrb = (FAR struct tcp_wrbuffer_s *)entry;

          /* If the ACKed sequence number is greater than the start
           * sequence number of the write buffer, then some or all of
           * the write buffer has been ACKed.
           */

          if (TCP_SEQ_GT(ackno, TCP_WBSEQNO(wrb)))
            {
              /* Get the sequence number at the end of the data */

              lastseq = TCP_WBSEQNO(wrb) + TCP_WBPKTLEN(wrb);
              ninfo("ACK: wrb=%p seqno=%" PRIu32
                    " lastseq=%" PRIu32 " pktlen=%u ackno=%" PRIu32 "\n",
                    wrb, TCP_WBSEQNO(wrb), lastseq, TCP_WBPKTLEN(wrb),
                    ackno);

              /* Has the entire buffer been ACKed? */

              if (TCP_SEQ_GTE(ackno, lastseq))
                {
                  ninfo("ACK: wrb=%p Freeing write buffer\n", wrb);

                  /* Yes... Remove the write buffer from ACK waiting queue */

                  sq_rem(entry, &conn->unacked_q);

                  /* And return the write buffer to the pool of free
                   * buffers
                   */

                  tcp_wrbuffer_release(wrb);

                  /* Notify any waiters if the write buffers have been
                   * drained.
                   */

                  psock_writebuffer_notify(conn);
                }
              else
                {
                  unsigned int trimlen;

                  /* No, then just trim the ACKed bytes from the beginning
                   * of the write buffer.  This will free up some I/O buffers
                   * that can be reused while are still sending the last
                   * buffers in the chain.
                   */

                  trimlen = TCP_SEQ_SUB(ackno, TCP_WBSEQNO(wrb));
                  if (trimlen > TCP_WBSENT(wrb))
                    {
                      /* More data has been ACKed then we have sent? */

                      trimlen = TCP_WBSENT(wrb);
                    }

                  ninfo("ACK: wrb=%p trim %u bytes\n", wrb, trimlen);

                  TCP_WBTRIM(wrb, trimlen);
                  TCP_WBSEQNO(wrb) += trimlen;
                  TCP_WBSENT(wrb) -= trimlen;

                  /* Set the new sequence number for what remains */

                  ninfo("ACK: wrb=%p seqno=%" PRIu32 " pktlen=%u\n",
                        wrb, TCP_WBSEQNO(wrb), TCP_WBPKTLEN(wrb));
                }
            }
          else if (ackno == TCP_WBSEQNO(wrb))
            {
              /* Reset the duplicate ack counter */

              if ((flags & TCP_NEWDATA) != 0)
                {
                  TCP_WBNACK(wrb) = 0;
                }

              /* Duplicate ACK? Retransmit data if need */

              if (++TCP_WBNACK(wrb) == TCP_FAST_RETRANSMISSION_THRESH)
                {
#ifdef CONFIG_NET_TCP_SELECTIVE_ACK
                  if ((conn->flags & TCP_SACK) &&
                      (tcp->tcpoffset & 0xf0) > 0x50)
                    {
                      /* Parse s-ack from tcp options */

                      nsacks = parse_sack(conn, tcp, ofosegs);

                      flags |= TCP_REXMIT;
                    }
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
                  else
#endif
#endif
                    {
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
                      /* Do fast retransmit */

                      rexmitno = ackno;
#endif

                      /* Reset counter */

                      TCP_WBNACK(wrb) = 0;
                    }
                }
            }
        }

      /* A special case is the head of the write_q which may be partially
       * sent and so can still have un-ACKed bytes that could get ACKed
       * before the entire write buffer has even been sent.
       */

      wrb = (FAR struct tcp_wrbuffer_s *)sq_peek(&conn->write_q);
      if (wrb && TCP_WBSENT(wrb) > 0 && TCP_SEQ_GT(ackno, TCP_WBSEQNO(wrb)))
        {
          uint32_t nacked;

          /* Number of bytes that were ACKed */

          nacked = TCP_SEQ_SUB(ackno, TCP_WBSEQNO(wrb));
          if (nacked > TCP_WBSENT(wrb))
            {
              /* More data has been ACKed then we have sent? ASSERT? */

              nacked = TCP_WBSENT(wrb);
            }

          ninfo("ACK: wrb=%p seqno=%" PRIu32
                " nacked=%" PRIu32 " sent=%u ackno=%" PRIu32 "\n",
                wrb, TCP_WBSEQNO(wrb), nacked, TCP_WBSENT(wrb), ackno);

          /* Trim the ACKed bytes from the beginning of the write buffer. */

          TCP_WBTRIM(wrb, nacked);
          TCP_WBSEQNO(wrb) += nacked;
          TCP_WBSENT(wrb) -= nacked;

          ninfo("ACK: wrb=%p seqno=%" PRIu32 " pktlen=%u sent=%u\n",
                wrb, TCP_WBSEQNO(wrb), TCP_WBPKTLEN(wrb), TCP_WBSENT(wrb));
        }
    }

  /* Check for a loss of connection */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      ninfo("Lost connection: %04x\n", flags);

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Report not connected */

          tcp_lost_connection(conn, conn->sndcb, flags);
        }

      /* Free write buffers and terminate polling */

      psock_lost_connection(conn, !!(flags & NETDEV_DOWN));
      return flags;
    }

#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
  if (rexmitno != 0)
    {
      FAR struct tcp_wrbuffer_s *wrb;
      FAR sq_entry_t *entry;
      FAR sq_entry_t *next;
      size_t sndlen;

      /* According to RFC 6298 (5.4), retransmit the earliest segment
       * that has not been acknowledged by the TCP receiver.
       */

      for (entry = sq_peek(&conn->unacked_q); entry; entry = next)
        {
          wrb = (FAR struct tcp_wrbuffer_s *)entry;
          next = sq_next(entry);

          if (rexmitno != TCP_WBSEQNO(wrb))
            {
              continue;
            }

          /* Reconstruct the length of the earliest segment to be
           * retransmitted.
           */

          sndlen = TCP_WBPKTLEN(wrb);

          if (sndlen > conn->mss)
            {
              sndlen = conn->mss;
            }

          /* As we are retransmitting, the sequence number is expected
           * already set for this write buffer.
           */

          DEBUGASSERT(TCP_WBSEQNO(wrb) != (unsigned)-1);

#ifdef NEED_IPDOMAIN_SUPPORT
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          send_ipselect(dev, conn);
#endif
          /* Then set-up to send that amount of data. (this won't actually
           * happen until the polling cycle completes).
           */

          tcp_setsequence(conn->sndseq, TCP_WBSEQNO(wrb));

          devif_iob_send(dev, TCP_WBIOB(wrb), sndlen,
                         0, tcpip_hdrsize(conn));
          if (dev->d_sndlen == 0)
            {
              return flags;
            }

          /* Reset the retransmission timer. */

          tcp_update_retrantimer(conn, conn->rto);

          /* Continue waiting */

          return flags;
        }
    }
#endif

#ifdef CONFIG_NET_TCP_SELECTIVE_ACK

  /* Check if we are being asked to retransmit s-ack data */

  if (nsacks > 0)
    {
      FAR struct tcp_wrbuffer_s *wrb;
      FAR sq_entry_t *entry;
      FAR sq_entry_t *next;
      uint32_t right;
      int i;

      /* Dump s-ack edge */

      for (i = 0, right = 0; i < nsacks; i++)
        {
          ninfo("TCP SACK [%d]"
                "[%" PRIu32 " : %" PRIu32 " : %" PRIu32 "]\n",
                i, ofosegs[i].left, ofosegs[i].right,
                TCP_SEQ_SUB(ofosegs[i].right, ofosegs[i].left));
        }

      for (entry = sq_peek(&conn->unacked_q); entry; entry = next)
        {
          wrb  = (FAR struct tcp_wrbuffer_s *)entry;
          next = sq_next(entry);

          for (i = 0, right = 0; i < nsacks; i++)
            {
              /* Wrb seqno out of s-ack edge ? do retransmit ! */

              if (TCP_SEQ_LT(TCP_WBSEQNO(wrb), ofosegs[i].left) &&
                  TCP_SEQ_GTE(TCP_WBSEQNO(wrb), right))
                {
                  ninfo("TCP REXMIT "
                        "[%" PRIu32 " : %" PRIu32 " : %d]\n",
                        TCP_WBSEQNO(wrb),
                        TCP_SEQ_ADD(TCP_WBSEQNO(wrb), TCP_WBPKTLEN(wrb)),
                        TCP_WBPKTLEN(wrb));
                  sq_rem(entry, &conn->unacked_q);
                  retransmit_segment(conn, (FAR void *)entry);
                  break;
                }

              right = ofosegs[i].right;
            }
        }
    }
  else
#endif

  /* Check if we are being asked to retransmit data */

  if ((flags & TCP_REXMIT) != 0)
    {
      FAR struct tcp_wrbuffer_s *wrb;
      FAR sq_entry_t *entry;

      ninfo("REXMIT: %04x\n", flags);

      /* If there is a partially sent write buffer at the head of the
       * write_q?  Has anything been sent from that write buffer?
       */

      wrb = (FAR struct tcp_wrbuffer_s *)sq_peek(&conn->write_q);
      ninfo("REXMIT: wrb=%p sent=%u\n", wrb, wrb ? TCP_WBSENT(wrb) : 0);

      if (wrb != NULL && TCP_WBSENT(wrb) > 0)
        {
          FAR struct tcp_wrbuffer_s *tmp;
          uint16_t sent;

          /* Yes.. Reset the number of bytes sent sent from
           * the write buffer
           */

          sent = TCP_WBSENT(wrb);
          if (conn->tx_unacked > sent)
            {
              conn->tx_unacked -= sent;
            }
          else
            {
              conn->tx_unacked = 0;
            }

          if (conn->sent > sent)
            {
              conn->sent -= sent;
            }
          else
            {
              conn->sent = 0;
            }

          TCP_WBSENT(wrb) = 0;
          ninfo("REXMIT: wrb=%p sent=%u, "
                "conn tx_unacked=%" PRId32 " sent=%" PRId32 "\n",
                wrb, TCP_WBSENT(wrb), conn->tx_unacked, conn->sent);

          /* Increment the retransmit count on this write buffer. */

          if (++TCP_WBNRTX(wrb) >= TCP_MAXRTX)
            {
              nwarn("WARNING: Expiring wrb=%p nrtx=%u\n",
                    wrb, TCP_WBNRTX(wrb));

              /* The maximum retry count as been exhausted. Remove the write
               * buffer at the head of the queue.
               */

              tmp = (FAR struct tcp_wrbuffer_s *)sq_remfirst(&conn->write_q);
              DEBUGASSERT(tmp == wrb);
              UNUSED(tmp);

              /* And return the write buffer to the free list */

              tcp_wrbuffer_release(wrb);

              /* Notify any waiters if the write buffers have been
               * drained.
               */

              psock_writebuffer_notify(conn);

              /* NOTE expired is different from un-ACKed, it is designed to
               * represent the number of segments that have been sent,
               * retransmitted, and un-ACKed, if expired is not zero, the
               * connection will be closed.
               *
               * field expired can only be updated at TCP_ESTABLISHED state
               */

              conn->expired++;
            }
        }

      /* Move all segments that have been sent but not ACKed to the write
       * queue again note, the un-ACKed segments are put at the head of the
       * write_q so they can be resent as soon as possible.
       */

      while ((entry = sq_remlast(&conn->unacked_q)) != NULL)
        {
          retransmit_segment(conn, (FAR void *)entry);
        }
    }

#if CONFIG_NET_SEND_BUFSIZE > 0
  /* Notify the send buffer available if wrbbuffer drained */

  tcp_sendbuffer_notify(conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */

  /* Check if the outgoing packet is available (it may have been claimed
   * by a sendto event serving a different thread).
   */

  if (dev->d_sndlen > 0)
    {
      /* Another thread has beat us sending data, wait for the next poll */

      return flags;
    }

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocessed incoming data or window size is zero.  In that event, we
   * will have to wait for the next polling cycle.
   */

  if ((conn->tcpstateflags & TCP_ESTABLISHED) &&
      ((flags & TCP_NEWDATA) == 0) &&
      (flags & (TCP_POLL | TCP_REXMIT | TCP_ACKDATA)) &&
      !(sq_empty(&conn->write_q)) &&
      conn->snd_wnd > 0)
    {
      FAR struct tcp_wrbuffer_s *wrb;
      uint32_t predicted_seqno;
      uint32_t seq;
      uint32_t snd_wnd_edge;
      size_t sndlen;

      /* Peek at the head of the write queue (but don't remove anything
       * from the write queue yet).  We know from the above test that
       * the write_q is not empty.
       */

      wrb = (FAR struct tcp_wrbuffer_s *)sq_peek(&conn->write_q);
      DEBUGASSERT(wrb);

      /* Set the sequence number for this segment.  If we are
       * retransmitting, then the sequence number will already
       * be set for this write buffer.
       */

      if (TCP_WBSEQNO(wrb) == (unsigned)-1)
        {
          TCP_WBSEQNO(wrb) = conn->isn + conn->sent;
        }

      /* Get the amount of data that we can send in the next packet.
       * We will send either the remaining data in the buffer I/O
       * buffer chain, or as much as will fit given the MSS and current
       * window size.
       */

      seq = TCP_WBSEQNO(wrb) + TCP_WBSENT(wrb);
      snd_wnd_edge = conn->snd_wl2 + conn->snd_wnd;
      if (TCP_SEQ_LT(seq, snd_wnd_edge))
        {
          uint32_t remaining_snd_wnd;

          sndlen = TCP_WBPKTLEN(wrb) - TCP_WBSENT(wrb);
          if (sndlen > conn->mss)
            {
              sndlen = conn->mss;
            }

          remaining_snd_wnd = TCP_SEQ_SUB(snd_wnd_edge, seq);
          if (sndlen > remaining_snd_wnd)
            {
              sndlen = remaining_snd_wnd;
            }

          ninfo("SEND: wrb=%p seq=%" PRIu32 " pktlen=%u sent=%u sndlen=%zu "
                "mss=%u snd_wnd=%u seq=%" PRIu32
                " remaining_snd_wnd=%" PRIu32 "\n",
                wrb, TCP_WBSEQNO(wrb), TCP_WBPKTLEN(wrb), TCP_WBSENT(wrb),
                sndlen, conn->mss,
                conn->snd_wnd, seq, remaining_snd_wnd);

          /* The TCP stack updates sndseq on receipt of ACK *before*
           * this function is called. In that case sndseq will point
           * to the next unacknowledged byte (which might have already
           * been sent). We will overwrite the value of sndseq here
           * before the packet is sent.
           */

          tcp_setsequence(conn->sndseq, TCP_WBSEQNO(wrb) + TCP_WBSENT(wrb));

    #ifdef NEED_IPDOMAIN_SUPPORT
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          send_ipselect(dev, conn);
    #endif
          /* Then set-up to send that amount of data with the offset
           * corresponding to the amount of data already sent. (this
           * won't actually happen until the polling cycle completes).
           */

          devif_iob_send(dev, TCP_WBIOB(wrb), sndlen,
                         TCP_WBSENT(wrb), tcpip_hdrsize(conn));
          if (dev->d_sndlen == 0)
            {
              return flags;
            }

          /* Remember how much data we send out now so that we know
           * when everything has been acknowledged.  Just increment
           * the amount of data sent. This will be needed in sequence
           * number calculations.
           */

          conn->tx_unacked += sndlen;
          conn->sent       += sndlen;

          /* Below prediction will become true,
           * unless retransmission occurrence
           */

          predicted_seqno = tcp_getsequence(conn->sndseq) + sndlen;

          if (TCP_SEQ_GT(predicted_seqno, conn->sndseq_max))
            {
               conn->sndseq_max = predicted_seqno;
            }

          ninfo("SEND: wrb=%p nrtx=%u tx_unacked=%" PRIu32
                " sent=%" PRIu32 "\n",
                wrb, TCP_WBNRTX(wrb), conn->tx_unacked, conn->sent);

          /* Increment the count of bytes sent from this write buffer */

          TCP_WBSENT(wrb) += sndlen;

          ninfo("SEND: wrb=%p sent=%u pktlen=%u\n",
                wrb, TCP_WBSENT(wrb), TCP_WBPKTLEN(wrb));

          /* Remove the write buffer from the write queue if the
           * last of the data has been sent from the buffer.
           */

          DEBUGASSERT(TCP_WBSENT(wrb) <= TCP_WBPKTLEN(wrb));
          if (TCP_WBSENT(wrb) >= TCP_WBPKTLEN(wrb))
            {
              FAR struct tcp_wrbuffer_s *tmp;

              ninfo("SEND: wrb=%p Move to unacked_q\n", wrb);

              tmp = (FAR struct tcp_wrbuffer_s *)sq_remfirst(&conn->write_q);
              DEBUGASSERT(tmp == wrb);
              UNUSED(tmp);

              /* Put the I/O buffer chain in the un-acked queue; the
               * segment is waiting for ACK again
               */

              psock_insert_segment(wrb, &conn->unacked_q);
            }

          /* Only one data can be sent by low level driver at once,
           * tell the caller stop polling the other connection.
           */

          flags &= ~TCP_POLL;
        }
    }

  /* Continue waiting */

  return flags;
}

/****************************************************************************
 * Name: tcp_max_wrb_size
 *
 * Description:
 *   Calculate the desired amount of data for a single
 *   struct tcp_wrbuffer_s.
 *
 ****************************************************************************/

static uint32_t tcp_max_wrb_size(FAR struct tcp_conn_s *conn)
{
  const uint32_t mss = conn->mss;
  uint32_t size;

  /* a few segments should be fine */

  size = 4 * mss;

  /* but it should not hog too many IOB buffers */

  if (size > CONFIG_IOB_NBUFFERS * CONFIG_IOB_BUFSIZE / 2)
    {
      size = CONFIG_IOB_NBUFFERS * CONFIG_IOB_BUFSIZE / 2;
    }

  /* also, we prefer a multiple of mss */

  if (size > mss)
    {
      const uint32_t odd = size % mss;
      size -= odd;
    }

  DEBUGASSERT(size > 0);
  ninfo("tcp_max_wrb_size = %" PRIu32 " for conn %p\n", size, conn);
  return size;
}

/****************************************************************************
 * Name: tcp_send_gettimeout
 *
 * Description:
 *   Calculate the send timeout
 *
 ****************************************************************************/

static unsigned int tcp_send_gettimeout(clock_t start, unsigned int timeout)
{
  unsigned int elapse;

  if (timeout != UINT_MAX)
    {
      elapse = TICK2MSEC(clock_systime_ticks() - start);
      if (elapse >= timeout)
        {
          timeout = 0;
        }
      else
        {
          timeout -= elapse;
        }
    }

  return timeout;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_tcp_send
 *
 * Description:
 *   psock_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 ****************************************************************************/

ssize_t psock_tcp_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len, int flags)
{
  FAR struct tcp_conn_s *conn;
  FAR struct tcp_wrbuffer_s *wrb;
  FAR const uint8_t *cp;
  unsigned int timeout;
  ssize_t    result = 0;
  bool       nonblock;
  int        ret = OK;
  clock_t    start;

  if (psock == NULL || psock->s_type != SOCK_STREAM ||
      psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      ret = -EBADF;
      goto errout;
    }

  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: Not connected\n");
      ret = -ENOTCONN;
      goto errout;
    }

  /* Make sure that we have the IP address mapping */

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (psock->s_domain == PF_INET)
#endif
    {
      /* Make sure that the IP address mapping is in the ARP table */

      ret = arp_send(conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_ARP_SEND */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      /* Make sure that the IP address mapping is in the Neighbor Table */

      ret = icmpv6_neighbor(conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      ret = -ENETUNREACH;
      goto errout;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
                            (flags & MSG_DONTWAIT) != 0;
  start    = clock_systime_ticks();
  timeout  = _SO_TIMEOUT(conn->sconn.s_sndtimeo);

  /* Dump the incoming buffer */

  BUF_DUMP("psock_tcp_send", buf, len);

  cp = buf;
  while (len > 0)
    {
      uint32_t max_wrb_size;
      unsigned int off;
      size_t chunk_len = len;
      ssize_t chunk_result;

      net_lock();

      /* Now that we have the network locked, we need to check the connection
       * state again to ensure the connection is still valid.
       */

      if (!_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          nerr("ERROR: No longer connected\n");
          ret = -ENOTCONN;
          goto errout_with_lock;
        }

      /* Allocate resources to receive a callback */

      if (conn->sndcb == NULL)
        {
          conn->sndcb = tcp_callback_alloc(conn);

          /* Test if the callback has been allocated */

          if (conn->sndcb == NULL)
            {
              /* A buffer allocation error occurred */

              nerr("ERROR: Failed to allocate callback\n");
              ret = nonblock ? -EAGAIN : -ENOMEM;
              goto errout_with_lock;
            }
        }

      /* Set up the callback in the connection */

      conn->sndcb->flags = (TCP_ACKDATA | TCP_REXMIT | TCP_POLL |
                               TCP_DISCONN_EVENTS);
      conn->sndcb->priv  = (FAR void *)conn;
      conn->sndcb->event = psock_send_eventhandler;

#if CONFIG_NET_SEND_BUFSIZE > 0
      /* If the send buffer size exceeds the send limit,
       * wait for the write buffer to be released
       */

      while (tcp_wrbuffer_inqueue_size(conn) >= conn->snd_bufs)
        {
          if (nonblock)
            {
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          ret = net_sem_timedwait_uninterruptible(&conn->snd_sem,
            tcp_send_gettimeout(start, timeout));
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  ret = -EAGAIN;
                }

              goto errout_with_lock;
            }
        }
#endif /* CONFIG_NET_SEND_BUFSIZE */

      while (true)
        {
          struct iob_s *iob;

          /* Allocate a write buffer.  Careful, the network will be
           * momentarily unlocked here.
           */

          /* Try to coalesce into the last wrb.
           *
           * But only when it might yield larger segments.
           * (REVISIT: It might make sense to lift this condition.
           * IOB boundaries and segment boundaries usually do not match.
           * It makes sense to save the number of IOBs.)
           *
           * Also, for simplicity, do it only when we haven't sent anything
           * from the the wrb yet.
           */

          max_wrb_size = tcp_max_wrb_size(conn);
          wrb = (FAR struct tcp_wrbuffer_s *)sq_tail(&conn->write_q);
          if (wrb != NULL && TCP_WBSENT(wrb) == 0 && TCP_WBNRTX(wrb) == 0 &&
              TCP_WBPKTLEN(wrb) < max_wrb_size &&
              (TCP_WBPKTLEN(wrb) % conn->mss) != 0)
            {
              wrb = (FAR struct tcp_wrbuffer_s *)sq_remlast(&conn->write_q);
              ninfo("coalesce %zu bytes to wrb %p (%" PRIu16 ")\n", len, wrb,
                    TCP_WBPKTLEN(wrb));
              DEBUGASSERT(TCP_WBPKTLEN(wrb) > 0);
            }
          else if (nonblock)
            {
              wrb = tcp_wrbuffer_tryalloc();
              ninfo("new wrb %p (non blocking)\n", wrb);
            }
          else
            {
              wrb = tcp_wrbuffer_timedalloc(tcp_send_gettimeout(start,
                                                                timeout));
              ninfo("new wrb %p\n", wrb);
            }

          if (wrb == NULL)
            {
              /* A buffer allocation error occurred */

              nerr("ERROR: Failed to allocate write buffer\n");

              if (nonblock || timeout != UINT_MAX)
                {
                  ret = -EAGAIN;
                }
              else
                {
                  ret = -ENOMEM;
                }

              goto errout_with_lock;
            }

          /* Initialize the write buffer */

          TCP_WBSEQNO(wrb) = (unsigned)-1;
          TCP_WBNRTX(wrb)  = 0;

          off = TCP_WBPKTLEN(wrb);
          if (off + chunk_len > max_wrb_size)
            {
              chunk_len = max_wrb_size - off;
            }

          /* Copy the user data into the write buffer.  We cannot wait for
           * buffer space.
           */

          /* The return value from TCP_WBTRYCOPYIN is either OK or
           * -ENOMEM if less than the entire data chunk could be allocated.
           * If -ENOMEM is returned, check if at least a part of the data
           * chunk was allocated. If more than zero bytes were sent
           * we return that number and let the caller deal with sending the
           * remaining data.
           */

          chunk_result = TCP_WBTRYCOPYIN(wrb, cp, chunk_len, off);
          if (chunk_result == -ENOMEM)
            {
              if (TCP_WBPKTLEN(wrb) > 0)
                {
                  DEBUGASSERT(TCP_WBPKTLEN(wrb) >= off);
                  chunk_result = TCP_WBPKTLEN(wrb) - off;
                  ninfo("INFO: Allocated part of the requested data "
                        "%zd/%zu\n",
                        chunk_result, chunk_len);

                  /* Note: chunk_result here can be 0 if we are trying
                   * to coalesce into the existing buffer and we failed
                   * to add anything.
                   */

                  DEBUGASSERT(chunk_result >= 0);
                }
              else
                {
                  chunk_result = 0;
                }
            }
          else
            {
              DEBUGASSERT(chunk_result == chunk_len);
            }

          if (chunk_result > 0)
            {
              break;
            }

          /* release wrb */

          if (TCP_WBPKTLEN(wrb) > 0)
            {
              DEBUGASSERT(TCP_WBSENT(wrb) == 0);
              DEBUGASSERT(TCP_WBPKTLEN(wrb) > 0);
              sq_addlast(&wrb->wb_node, &conn->write_q);
            }
          else
            {
              tcp_wrbuffer_release(wrb);
            }

          if (nonblock || (timeout != UINT_MAX &&
                           tcp_send_gettimeout(start, timeout) == 0))
            {
              nerr("ERROR: Failed to add data to the I/O chain\n");
              ret = -EAGAIN;
              goto errout_with_lock;
            }

          /* Wait for at least one IOB getting available.
           *
           * Note: net_ioballoc releases the network lock when blocking.
           * It allows our write_q being drained in the meantime. Otherwise,
           * we risk a deadlock with other threads competing on IOBs.
           */

          iob = net_iobtimedalloc(true, tcp_send_gettimeout(start, timeout));
          if (iob != NULL)
            {
              iob_free_chain(iob);
            }
        }

      /* Dump I/O buffer chain */

      TCP_WBDUMP("I/O buffer chain", wrb, TCP_WBPKTLEN(wrb), 0);

      /* psock_send_eventhandler() will send data in FIFO order from the
       * conn->write_q
       */

      sq_addlast(&wrb->wb_node, &conn->write_q);
      ninfo("Queued WRB=%p pktlen=%u write_q(%p,%p)\n",
            wrb, TCP_WBPKTLEN(wrb),
            conn->write_q.head, conn->write_q.tail);

      /* Notify the device driver of the availability of TX data */

      tcp_send_txnotify(psock, conn);
      net_unlock();

      if (chunk_result == 0)
        {
          DEBUGASSERT(nonblock);
          if (result == 0)
            {
              result = -EAGAIN;
            }

          break;
        }

      if (chunk_result < 0)
        {
          if (result == 0)
            {
              result = chunk_result;
            }

          break;
        }

      DEBUGASSERT(chunk_result <= len);
      DEBUGASSERT(chunk_result <= chunk_len);
      DEBUGASSERT(result >= 0);
      cp += chunk_result;
      len -= chunk_result;
      result += chunk_result;
    }

  /* Check for errors.  Errors are signaled by negative errno values
   * for the send length
   */

  if (result < 0)
    {
      ret = result;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return result;

errout_with_lock:
  net_unlock();

errout:
  if (result > 0)
    {
      return result;
    }

  return ret;
}

/****************************************************************************
 * Name: psock_tcp_cansend
 *
 * Description:
 *   psock_tcp_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   conn     The TCP connection of interest
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *   -ENOTCONN
 *     The socket is not connected.
 *
 ****************************************************************************/

int psock_tcp_cansend(FAR struct tcp_conn_s *conn)
{
  /* Verify that we received a valid socket */

  if (!conn)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* Verify that this is connected TCP socket */

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: Not connected\n");
      return -ENOTCONN;
    }

  /* In order to setup the send, we need to have at least one free write
   * buffer head and at least one free IOB to initialize the write buffer
   * head.
   *
   * REVISIT:  The send will still block if we are unable to buffer
   * the entire user-provided buffer which may be quite large.
   * We will almost certainly need to have more than one free IOB,
   * but we don't know how many more.
   */

  if (tcp_wrbuffer_test() < 0 || iob_navail(true) <= 0)
    {
      return -EWOULDBLOCK;
    }

  return OK;
}

/****************************************************************************
 * Name: tcp_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
void tcp_sendbuffer_notify(FAR struct tcp_conn_s *conn)
{
  int val = 0;

  nxsem_get_value(&conn->snd_sem, &val);
  if (val < 0)
    {
      nxsem_post(&conn->snd_sem);
    }
}
#endif /* CONFIG_NET_SEND_BUFSIZE */

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCP_WRITE_BUFFERS */
