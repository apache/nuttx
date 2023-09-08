/****************************************************************************
 * net/tcp/tcp_callback.c
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
#include <string.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "tcp/tcp.h"
#include "utils/utils.h"

#ifdef NET_TCP_HAVE_STACK

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_data_event
 *
 * Description:
 *   Handle data that is not accepted by the application because there is no
 *   listener in place ready to receive the data.
 *
 * Assumptions:
 * - The caller has checked that TCP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

static inline uint16_t
tcp_data_event(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
               uint16_t flags)
{
  uint16_t recvlen;

  /* Assume that we will ACK the data.  The data will be ACKed if it is
   * placed in the read-ahead buffer -OR- if it zero length
   */

  flags = (flags & ~TCP_NEWDATA) | TCP_SNDACK;

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with TCP_NEWDATA set just to cause an ACK).
   */

  if (dev->d_len > 0)
    {
      ninfo("No listener on connection\n");

      /* Save as the packet data as in the read-ahead buffer.  NOTE that
       * partial packets will not be buffered.
       */

      recvlen = tcp_datahandler(dev, conn,
                                (dev->d_appdata - dev->d_iob->io_data) -
                                dev->d_iob->io_offset);

      net_incr32(conn->rcvseq, recvlen);
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return flags;
}

/****************************************************************************
 * Name: tcp_ofoseg_data_event
 *
 * Description:
 *   Handle out-of-order segment to readahead poll.
 *
 * Assumptions:
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_OUT_OF_ORDER
static uint16_t tcp_ofoseg_data_event(FAR struct net_driver_s *dev,
                                      FAR struct tcp_conn_s *conn,
                                      uint16_t flags)
{
  FAR struct tcp_ofoseg_s *seg;
  uint32_t rcvseq;
  int i = 0;

  /* Assume that we will ACK the data.  The data will be ACKed if it is
   * placed in the read-ahead buffer -OR- if it zero length
   */

  flags |= TCP_SNDACK;

  /* Get the receive sequence number */

  rcvseq = tcp_getsequence(conn->rcvseq);

  ninfo("TCP OFOSEG rcvseq [%" PRIu32 "]\n", rcvseq);

  /* Foreach out-of-order segments */

  while (i < conn->nofosegs)
    {
      seg = &conn->ofosegs[i];

      /* rcvseq -->|
       * ofoseg    |------|
       */

      if (rcvseq == seg->left)
        {
          ninfo("TCP OFOSEG input [%" PRIu32 " : %" PRIu32 " : %u]\n",
                 seg->left, seg->right, seg->data->io_pktlen);
          rcvseq = TCP_SEQ_ADD(rcvseq,
                               seg->data->io_pktlen);
          net_incr32(conn->rcvseq, seg->data->io_pktlen);
          net_iob_concat(&conn->readahead, &seg->data);
        }
      else if (TCP_SEQ_GT(rcvseq, seg->left))
        {
          /* rcvseq       -->|
           * ofoseg  |------|
           */

          if (TCP_SEQ_GTE(rcvseq, seg->right))
            {
              /* Remove stale segments */

              iob_free_chain(seg->data);
              seg->data = NULL;
            }

          /* rcvseq  -->|
           * ofoseg   |------|
           */

          else
            {
              seg->data =
                iob_trimhead(seg->data,
                             TCP_SEQ_SUB(rcvseq, seg->left));
              seg->left = rcvseq;
              if (seg->data != NULL)
                {
                  ninfo("TCP OFOSEG input "
                        "[%" PRIu32 " : %" PRIu32 " : %u]\n",
                        seg->left, seg->right, seg->data->io_pktlen);
                  rcvseq = TCP_SEQ_ADD(rcvseq,
                                       seg->data->io_pktlen);
                  net_incr32(conn->rcvseq, seg->data->io_pktlen);
                  net_iob_concat(&conn->readahead, &seg->data);
                }
            }
        }

      /* Rebuild out-of-order pool if segment is consumed */

      if (seg->data == NULL)
        {
          for (; i < conn->nofosegs - 1; i++)
            {
              conn->ofosegs[i] = conn->ofosegs[i + 1];
            }

          conn->nofosegs--;

          /* Try segments again */

          i = 0;
        }
      else
        {
          i++;
        }
    }

  return flags;
}
#endif /* CONFIG_NET_TCP_OUT_OF_ORDER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_ofoseg_bufsize
 *
 * Description:
 *   Calculate the pending size of out-of-order buffer
 *
 * Input Parameters:
 *   conn   - The TCP connection of interest
 *
 * Returned Value:
 *   Total size of out-of-order buffer
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_OUT_OF_ORDER
int tcp_ofoseg_bufsize(FAR struct tcp_conn_s *conn)
{
  int total = 0;
  int i;

  for (i = 0; i < conn->nofosegs; i++)
    {
      total += conn->ofosegs[i].data->io_pktlen;
    }

  return total;
}
#endif /* CONFIG_NET_TCP_OUT_OF_ORDER */

/****************************************************************************
 * Name: tcp_callback
 *
 * Description:
 *   Inform the application holding the TCP socket of a change in state.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t tcp_callback(FAR struct net_driver_s *dev,
                      FAR struct tcp_conn_s *conn, uint16_t flags)
{
#if defined(CONFIG_NET_TCP_NOTIFIER) || defined(CONFIG_NET_TCP_OUT_OF_ORDER)
  uint16_t orig = flags;
#endif

  /* Prepare device buffer */

  if (dev->d_iob == NULL && netdev_iob_prepare(dev, false, 0) != OK)
    {
      return 0;
    }

  /* Preserve the TCP_ACKDATA, TCP_CLOSE, and TCP_ABORT in the response.
   * These is needed by the network to handle responses and buffer state.
   * The TCP_NEWDATA indication will trigger the ACK response, but must be
   * explicitly set in the callback.
   */

  ninfo("flags: %04x\n", flags);

  /* Perform the data callback.  When a data callback is executed from
   * 'list', the input flags are normally returned, however, the
   * implementation may set one of the following:
   *
   *   TCP_CLOSE   - Gracefully close the current connection
   *   TCP_ABORT   - Abort (reset) the current connection on an error that
   *                 prevents TCP_CLOSE from working.
   *
   * And/Or set/clear the following:
   *
   *   TCP_NEWDATA - May be cleared to indicate that the data was consumed
   *                 and that no further process of the new data should be
   *                 attempted.
   *   TCP_SNDACK  - If TCP_NEWDATA is cleared, then TCP_SNDACK may be set
   *                 to indicate that an ACK should be included in the
   *                 response.  (In TCP_NEWDATA is cleared but TCP_SNDACK is
   *                 not set, then dev->d_len should also be cleared).
   */

  flags = devif_conn_event(dev, flags, conn->sconn.list);

  /* There may be no new data handler in place at them moment that the new
   * incoming data is received.  If the new incoming data was not handled,
   * then either (1) put the unhandled incoming data in the read-ahead
   * buffer (if enabled) or (2) suppress the ACK to the data in the hope
   * that it will be re-transmitted at a better time.
   */

  if ((flags & TCP_NEWDATA) != 0)
    {
      /* Data was not handled.. dispose of it appropriately */

      flags = tcp_data_event(dev, conn, flags);
    }

#ifdef CONFIG_NET_TCP_OUT_OF_ORDER
  if ((orig & TCP_NEWDATA) != 0 && conn->nofosegs > 0)
    {
      /* Try out-of-order pool if new data is coming */

      flags = tcp_ofoseg_data_event(dev, conn, flags);
    }
#endif

  /* Check if there is a connection-related event and a connection
   * callback.
   */

  if ((flags & TCP_CONN_EVENTS) != 0)
    {
      /* Perform the callback disconnect callbacks */

      flags = devif_conn_event(dev, flags, conn->connevents);
    }

#ifdef CONFIG_NET_TCP_NOTIFIER
  /* Provide notification(s) if the TCP connection has been lost. */

  if ((orig & TCP_DISCONN_EVENTS) != 0)
    {
      tcp_disconnect_signal(conn);
    }
#endif

  /* Re-prepare the device buffer if d_iob is consumed by the stack */

  if (dev->d_iob == NULL)
    {
      netdev_iob_prepare(dev, false, 0);
    }

  return flags;
}

/****************************************************************************
 * Name: tcp_datahandler
 *
 * Description:
 *   Handle data that is not accepted by the application.  This may be called
 *   either (1) from the data receive logic if it cannot buffer the data, or
 *   (2) from the TCP event logic is there is no listener in place ready to
 *   receive the data.
 *
 * Input Parameters:
 *   conn - A pointer to the TCP connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that TCP_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t tcp_datahandler(FAR struct net_driver_s *dev,
                         FAR struct tcp_conn_s *conn,
                         uint16_t offset)
{
  FAR struct iob_s *iob = dev->d_iob;
  uint16_t buflen;

  if (offset > 0)
    {
      /* Remove 'bufoff' bytes from the beginning of the input I/O chain */

      iob = iob_trimhead(iob, offset);
    }

  /* Trim tail if l3/l4 header has been removed */

  if (dev->d_len < iob->io_pktlen)
    {
      iob = iob_trimtail(iob, iob->io_pktlen - dev->d_len);
    }

  buflen = iob->io_pktlen;

  /* Concat the iob to readahead */

  net_iob_concat(&conn->readahead, &iob);

  /* Clear device buffer */

  netdev_iob_clear(dev);

#ifdef CONFIG_NET_TCP_NOTIFIER
  /* Provide notification(s) that additional TCP read-ahead data is
   * available.
   */

  tcp_readahead_signal(conn);
#endif

  ninfo("Buffered %" PRIu16 " bytes\n", buflen);
  return buflen;
}

#endif /* NET_TCP_HAVE_STACK */
