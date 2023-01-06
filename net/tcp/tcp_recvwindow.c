/****************************************************************************
 * net/tcp/tcp_recvwindow.c
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
#include <debug.h>

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

  niob_avail = iob_navail(true);

  /* Is there a a queue entry and IOBs available for read-ahead buffering? */

  if (niob_avail > 0)
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

      recvwndo = tailroom + (niob_avail * CONFIG_IOB_BUFSIZE);
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

  recvwndo = tcp_calc_rcvsize(conn, recvwndo);

#ifdef CONFIG_NET_TCP_OUT_OF_ORDER
  /* Calculate the minimum desired size */

  if (conn->nofosegs > 0)
    {
      uint32_t desire = conn->ofosegs[0].left -
                        tcp_getsequence(conn->rcvseq);
      int bufsize = tcp_ofoseg_bufsize(conn);

      if (desire < tcp_rx_mss(dev))
        {
          desire = tcp_rx_mss(dev);
        }

      if (TCP_SEQ_LT(recvwndo, bufsize))
        {
          recvwndo = 0;
        }
      else
        {
          recvwndo -= bufsize;
        }

      if (recvwndo < desire)
        {
          recvwndo = desire;
        }
    }
#endif /* CONFIG_NET_TCP_OUT_OF_ORDER */

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
