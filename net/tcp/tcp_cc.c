/****************************************************************************
 * net/tcp/tcp_cc.c
 * Handling TCP congestion control
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

#include <debug.h>

#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TCP_IPV4_DEFAULT_MSS 536

/* Initial Window threshold constants */

#define IW_MAX 4380          /* Initial Window maximum */
#define IW_MAX_HALF 2190
#define IW_MAX_QUATER 1095

/* Calculate the Initial Window, also used as Restart Window
 * RFC5681 Section 3.1 specifies the default conservative values.
 */

#define CC_INIT_CWND(cwnd, mss) \
 do { \
  if ((mss) > IW_MAX_HALF) \
    { \
      (cwnd) = 2 * (mss); \
    } \
  else if ((mss) > IW_MAX_QUATER) \
    { \
      (cwnd) = 3 * (mss); \
    } \
  else \
    { \
      (cwnd) = 4 * (mss); \
    } \
 } while(0)

/* Increments a size inc and holds at max value rather than rollover. */

#define CC_CWND_INC(wnd, inc) \
 do { \
  if ((uint32_t)((wnd) + (inc)) >= (wnd)) \
    { \
      (wnd) = (uint32_t)((wnd) + (inc)); \
    } \
  else \
    { \
      (wnd) = (uint32_t)-1; \
    } \
 } while(0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_cc_init
 *
 * Description:
 *   Initialize the congestion control variables, cwnd, ssthresh and dupacks.
 *   The function is called on starting a new connection.
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The normal user level code is calling the connect/accept to start a new
 *   connection.
 *
 ****************************************************************************/

void tcp_cc_init(FAR struct tcp_conn_s *conn)
{
  CC_INIT_CWND(conn->cwnd, conn->mss);

  /* RFC 5681 recommends setting ssthresh arbitrarily high and
   * gives an example of using the largest advertised receive window.
   * We've seen complications with receiving TCPs that use window
   * scaling and/or window auto-tuning where the initial advertised
   * window is very small and then grows rapidly once the connection
   * is established. To avoid these complications, we set ssthresh to
   * the largest effective cwnd (amount of in-flight data) that the
   * sender can have.
   */

  conn->ssthresh = 2 * TCP_IPV4_DEFAULT_MSS;
  conn->dupacks = 0;
}

/****************************************************************************
 * Name: tcp_cc_update
 *
 * Description:
 *   Update the congestion control variables when recieve the SYNACK/ACK
 *   packet from the peer in the connection phase.
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *   tcp    - The TCP header.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void tcp_cc_update(FAR struct tcp_conn_s *conn, FAR struct tcp_hdr_s *tcp)
{
  /* After Fast retransmitted, set ssthresh to the maximum of
   * the unacked and the 2*SMSS, and enter to Fast Recovery.
   * ssthresh = max (FlightSize / 2, 2*SMSS) referring to rfc5681
   * cwnd=ssthresh + 3*SMSS  referring to rfc5681
   */

  if (conn->flags & TCP_INFT)
    {
      conn->ssthresh = MAX(conn->tx_unacked / 2, 2 * conn->mss);
      conn->cwnd = conn->ssthresh + 3 * conn->mss;

      conn->flags &= ~TCP_INFT;
      conn->flags |= TCP_INFR;
    }

  /* Update the cc parameters in the TCP_SYN_RCVD and TCP_SYN_SENT states
   * when the tcp connection is established.
   */

  else
    {
      conn->last_ackno = tcp_getsequence(tcp->ackno);
      CC_INIT_CWND(conn->cwnd, conn->mss);
      conn->max_cwnd = conn->snd_wnd;
      conn->ssthresh = MAX(conn->snd_wnd, conn->ssthresh);
    }
}

/****************************************************************************
 * Name: tcp_cc_recv_ack
 *
 * Description:
 *   Update congestion control variables
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *   tcp    - The TCP header.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void tcp_cc_recv_ack(FAR struct tcp_conn_s *conn, FAR struct tcp_hdr_s *tcp)
{
  uint32_t ackno = tcp_getsequence(tcp->ackno);

  /* Its only a duplicate ack if:
   * 1) It doesn't ACK new data
   * 2) There is outstanding unacknowledged data (retransmission
   *    timer running)
   * 3) The ACK is == biggest ACK sequence number so far (last_ackno)
   *
   * If it passes all conditions, should process as a dupack:
   * a) dupacks < 3: do nothing
   * b) dupacks == 3: fast retransmit
   * c) dupacks > 3: increase cwnd
   *
   * If ackno is between last_ackno and snd_seq, should reset dupack counter.
   */

  /* Clause 1 */

  if (TCP_SEQ_LTE(ackno, conn->last_ackno))
    {
      /* Clause 2 and Clause 3 */

      if (conn->timer >= 0 &&
          conn->last_ackno == ackno)
        {
          if (++conn->dupacks > TCP_FAST_RETRANSMISSION_THRESH)
            {
              /* Inflate the congestion window */

              CC_CWND_INC(conn->cwnd, conn->mss);
            }

          if (conn->dupacks >= TCP_FAST_RETRANSMISSION_THRESH)
            {
              /* Do fast retransmit, but it is delayed in
               * psock_send_eventhandler. Set the TCP_INFT flag.
               */

              conn->flags |= TCP_INFT;
              conn->fr_recover = tcp_getsequence(conn->sndseq);
            }
        }
    }
  else if (TCP_SEQ_GT(ackno, conn->last_ackno) &&
           TCP_SEQ_LTE(ackno, tcp_getsequence(conn->sndseq)))
    {
      /* We come here when the ACK acknowledges new data. */

      uint32_t acked = TCP_SEQ_SUB(ackno, conn->last_ackno);

      /* Reset dupacks and update last_ackno. */

      conn->dupacks = 0;
      conn->last_ackno = ackno;

      /* When the ackno covers more than the fr_recover, exit the
       * fast recovery. Then, reset the "IN Fast Recovery" flags.
       * Also reset the congestion window to the slow start threshold.
       * If not, cwnd should be increased by mss. RFC6582.
       */

      if (conn->flags & TCP_INFR)
        {
          if (ackno - 1 > conn->fr_recover)
            {
              /* Reset the fast retransmit variables. */

              conn->flags &= ~TCP_INFR;
              conn->cwnd = conn->ssthresh;
            }
          else
            {
              CC_CWND_INC(conn->cwnd, conn->mss);
              return;
            }
        }

      /* Update the congestion control variables (cwnd and ssthresh). */

      if (conn->tcpstateflags >= TCP_ESTABLISHED)
        {
          uint32_t increase;

          if (conn->cwnd < conn->ssthresh)
            {
              /* slow start (RFC 5681):
               * Grow cwnd exponentially by maxseg(smss) per ACK.
               */

              increase = acked > 0 ? MIN(acked, conn->mss) : conn->mss;

              CC_CWND_INC(conn->cwnd, increase);
              ninfo("update slow start cwnd to %u\n", conn->cwnd);
            }
          else
            {
              /* cong avoid (RFC 5681):
               * Grow cwnd linearly by approximately maxseg per RTT using
               * maxseg^2 / cwnd per ACK as the increment.
               * If cwnd > maxseg^2, fix the cwnd increment at 1 byte to
               * avoid capping cwnd.
               */

              increase = MAX((conn->mss * conn->mss / conn->cwnd), 1);

              CC_CWND_INC(conn->cwnd, increase);
              conn->cwnd = MIN(conn->cwnd, conn->max_cwnd);
              ninfo("update congestion avoidance cwnd to %u\n", conn->cwnd);
            }
        }
    }
}
