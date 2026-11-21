/****************************************************************************
 * net/tcp/tcp_recvwindow.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdbool.h>
#include <sys/param.h>
#include <nuttx/debug.h>

#include <net/if.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_calc_rcvsize
 *
 * Description:
 *   Calculate the possible max TCP receive buffer size for the connection.
 *
 * Input Parameters:
 *   conn     - The TCP connection.
 *   recvwndo - The TCP receive window size
 *
 * Returned Value:
 *   The value of the TCP receive buffer size.
 *
 ****************************************************************************/

static uint32_t tcp_calc_rcvsize(FAR struct tcp_conn_s *conn,
                                 uint32_t recvwndo)
{
#if CONFIG_NET_RECV_BUFSIZE > 0
  uint32_t recvsize;
  uint32_t desire;

  recvsize = conn->readahead ? conn->readahead->io_pktlen : 0;
  if (conn->rcv_bufs > recvsize)
    {
      desire = conn->rcv_bufs - recvsize;
      if (recvwndo > desire)
        {
          recvwndo = desire;
        }
    }
  else
    {
      recvwndo = 0;
    }
#endif

  return recvwndo;
}

/****************************************************************************
 * Name: tcp_promised_iobs
 *
 * Description:
 *   Return the IOBs needed by the receive windows advertised to the other
 *   connections.  A segment holds a whole frame of frameiobs IOBs.
 *
 ****************************************************************************/

static int tcp_promised_iobs(FAR struct tcp_conn_s *conn, uint16_t mss,
                             int frameiobs)
{
  FAR struct tcp_conn_s *other = NULL;
  uint32_t segs = 0;
  uint32_t rcvseq;
  uint8_t state;

  tcp_conn_list_lock();
  while ((other = tcp_nextconn(other)) != NULL)
    {
      state  = other->tcpstateflags & TCP_STATE_MASK;
      rcvseq = tcp_getsequence(other->rcvseq);
      if (other != conn &&
          (state == TCP_SYN_RCVD ||
           (state >= TCP_ESTABLISHED && state <= TCP_FIN_WAIT_2)) &&
          TCP_SEQ_GT(other->rcv_adv, rcvseq))
        {
          segs += (TCP_SEQ_SUB(other->rcv_adv, rcvseq) + mss - 1) / mss;
        }
    }

  tcp_conn_list_unlock();
  return segs * frameiobs;
}

/****************************************************************************
 * Name: tcp_maxrcvwin
 *
 * Description:
 *   Calculate the possible max TCP receive window for the connection.
 *
 * Input Parameters:
 *   conn - The TCP connection.
 *
 * Returned Value:
 *   The value of the TCP receive window.
 ****************************************************************************/

static uint32_t tcp_maxrcvwin(FAR struct tcp_conn_s *conn)
{
  uint32_t recvwndo;

  /* Calculate the max possible window size for the connection.
   * This needs to be in sync with tcp_get_recvwindow().
   */

  recvwndo = tcp_calc_rcvsize(conn, (CONFIG_IOB_NBUFFERS -
                                     CONFIG_IOB_THROTTLE) *
                                     CONFIG_IOB_BUFSIZE);
#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  recvwndo >>= conn->rcv_scale;
#endif

  if (recvwndo > UINT16_MAX)
    {
      recvwndo = UINT16_MAX;
    }

#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  recvwndo <<= conn->rcv_scale;
#endif

  return recvwndo;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_recvwindow_loss
 *
 * Description:
 *   A segment arrived after a gap: the frames in front of it were lost.
 *   Limit the receive window that the connection advertises.
 *
 * Input Parameters:
 *   dev  - The device that received the segment.
 *   conn - The TCP connection.
 *
 ****************************************************************************/

void tcp_recvwindow_loss(FAR struct net_driver_s *dev,
                         FAR struct tcp_conn_s *conn)
{
  uint32_t rcvseq = tcp_getsequence(conn->rcvseq);
  uint32_t mss = tcp_rx_mss(dev);
  uint32_t wnd = 0;

  /* The other segments of the same window are lost with the first one:
   * limit the window once per window.
   */

  if (conn->rcv_clamp != 0 && TCP_SEQ_LT(rcvseq, conn->rcv_lossseq))
    {
      return;
    }

  if (TCP_SEQ_GT(conn->rcv_adv, rcvseq))
    {
      wnd = TCP_SEQ_SUB(conn->rcv_adv, rcvseq);
    }

  if (conn->rcv_clamp != 0 && conn->rcv_clamp < wnd)
    {
      wnd = conn->rcv_clamp;
    }

  conn->rcv_clampmax = MAX(wnd > mss ? wnd - mss : 0, 2 * mss);
  conn->rcv_clamp    = MAX(wnd / 2, 2 * mss);
  conn->rcv_clampseq = conn->rcv_adv;
  conn->rcv_lossseq  = conn->rcv_adv;
}

/****************************************************************************
 * Name: tcp_get_recvwindow
 *
 * Description:
 *   Calculate the TCP receive window for the specified device.
 *
 * Input Parameters:
 *   dev  - The device whose TCP receive window will be updated.
 *   conn - The TCP connection structure holding connection information.
 *
 * Returned Value:
 *   The value of the TCP receive window to use.
 *
 ****************************************************************************/

uint32_t tcp_get_recvwindow(FAR struct net_driver_s *dev,
                            FAR struct tcp_conn_s *conn)
{
  uint32_t tailroom;
  uint32_t recvwndo;
  uint16_t mss;
  int frameiobs;
  int niob_avail;

  /* Update the TCP received window based on read-ahead I/O buffer
   * and IOB chain availability.
   * The amount of read-ahead
   * data that can be buffered is given by the number of IOBs available
   * (ignoring competition with other IOB consumers).
   */

  if (conn->readahead != NULL)
    {
      tailroom = iob_tailroom(conn->readahead);
    }
  else
    {
      tailroom = 0;
    }

  /* A segment holds a whole frame.  Keep the IOBs of the windows of the
   * other connections.
   */

  mss = tcp_rx_mss(dev);
  frameiobs = (CONFIG_NET_LL_GUARDSIZE + dev->d_pktsize +
               CONFIG_IOB_BUFSIZE - 1) / CONFIG_IOB_BUFSIZE;
  niob_avail = iob_navail(true) - tcp_promised_iobs(conn, mss, frameiobs);

  /* Is there a a queue entry and IOBs available for read-ahead buffering? */

  if (niob_avail >= frameiobs)
    {
      /* The optimal TCP window size is the amount of TCP data that we can
       * currently buffer via TCP read-ahead buffering for the device packet
       * buffer.  This logic here assumes that all IOBs are available for
       * TCP buffering.
       *
       * Assume that all of the available IOBs are can be used for buffering
       * on this connection.
       *
       * REVISIT:  In an environment with multiple, active read-ahead TCP
       * sockets (and perhaps multiple network devices) or if there are
       * other consumers of IOBs (such as for TCP write buffering) then the
       * total number of IOBs will all not be available for read-ahead
       * buffering for this connection.
       */

      recvwndo = tailroom + (niob_avail / frameiobs) * mss;
    }
#if CONFIG_IOB_THROTTLE > 0
  else if (conn->readahead == NULL)
    {
      /* Advertise maximum segment size for window edge if here is no
       * available iobs on current "free" connection.
       *
       * Note: hopefully, a single mss-sized packet can be queued by
       * the throttled=false case in tcp_datahandler().
       */

      int niob_avail_no_throttle = iob_navail(false);

      recvwndo = tcp_rx_mss(dev);
      if (recvwndo > niob_avail_no_throttle * CONFIG_IOB_BUFSIZE)
        {
          recvwndo = niob_avail_no_throttle * CONFIG_IOB_BUFSIZE;
        }
    }
#endif
  else /* niob_avail == 0 */
    {
      /* No IOBs are available.
       * Advertise the edge of window to zero.
       *
       * NOTE:  If no IOBs are available, then the next packet will be
       * lost if there is no listener on the connection.
       */

      recvwndo = tailroom;
    }

  /* After a loss the window grows by one segment per window, up to one
   * segment less than the window that lost frames.
   */

  if (conn->rcv_clamp != 0)
    {
      uint32_t rcvseq = tcp_getsequence(conn->rcvseq);

      if (TCP_SEQ_GTE(rcvseq, conn->rcv_clampseq) &&
          conn->rcv_clamp < conn->rcv_clampmax)
        {
          conn->rcv_clamp    = MIN(conn->rcv_clamp + mss,
                                   conn->rcv_clampmax);
          conn->rcv_clampseq = rcvseq + conn->rcv_clamp;
        }

      recvwndo = MIN(recvwndo, conn->rcv_clamp);
    }

  recvwndo = tcp_calc_rcvsize(conn, recvwndo);

#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  recvwndo >>= conn->rcv_scale;
#endif

  if (recvwndo > UINT16_MAX)
    {
      recvwndo = UINT16_MAX;
    }

#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  recvwndo <<= conn->rcv_scale;
#endif

  return recvwndo;
}

bool tcp_should_send_recvwindow(FAR struct tcp_conn_s *conn)
{
  FAR struct net_driver_s *dev = conn->dev;
  uint32_t win;
  uint32_t maxwin;
  uint32_t oldwin;
  uint32_t rcvseq;
  uint32_t adv;
  uint16_t mss;

  /* A socket that never connected has no device and no peer */

  if (dev == NULL)
    {
      return false;
    }

  /* Note: rcv_adv can be smaller than rcvseq.
   * For examples, when:
   *
   * - we shrunk the window
   * - zero window probes advanced rcvseq
   */

  rcvseq = tcp_getsequence(conn->rcvseq);
  if (TCP_SEQ_GT(conn->rcv_adv, rcvseq))
    {
      oldwin = TCP_SEQ_SUB(conn->rcv_adv, rcvseq);
    }
  else
    {
      oldwin = 0;
    }

  win = tcp_get_recvwindow(dev, conn);

  /* If the window doesn't extend, don't send. */

  if (win <= oldwin)
    {
      ninfo("Returning false: "
            "rcvseq=%" PRIu32 ", rcv_adv=%" PRIu32 ", "
            "old win=%" PRIu32 ", new win=%" PRIu32 "\n",
            rcvseq, conn->rcv_adv, oldwin, win);
      return false;
    }

  adv = win - oldwin;

  /* The following conditions are inspired from NetBSD TCP stack.
   *
   * - If we can extend the window by the half of the max possible size,
   *   send it.
   *
   * - If we can extend the window by 2 * mss, send it.
   */

  maxwin = tcp_maxrcvwin(conn);
  if (2 * adv >= maxwin)
    {
      ninfo("Returning true: "
            "adv=%" PRIu32 ", maxwin=%" PRIu32 "\n",
            adv, maxwin);
      return true;
    }

  /* Revisit: the real expected size should be used instead.
   * E.g. consider the path MTU
   */

  mss = tcp_rx_mss(dev);
  if (adv >= 2 * mss)
    {
      ninfo("Returning true: "
            "adv=%" PRIu32 ", mss=%" PRIu16 ", maxwin=%" PRIu32 "\n",
            adv, mss, maxwin);
      return true;
    }

  ninfo("Returning false: "
        "adv=%" PRIu32 ", mss=%" PRIu16 ", maxwin=%" PRIu32 "\n",
        adv, mss, maxwin);
  return false;
}
