/****************************************************************************
 * net/tcp/tcp_callback.c
 *
 *   Copyright (C) 2007-2009, 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "tcp/tcp.h"

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
  uint16_t ret;

  /* Assume that we will ACK the data.  The data will be ACKed if it is
   * placed in the read-ahead buffer -OR- if it zero length
   */

  ret = (flags & ~TCP_NEWDATA) | TCP_SNDACK;

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with TCP_NEWDATA set just to cause an ACK).
   */

  if (dev->d_len > 0)
    {
#ifdef CONFIG_NET_TCP_READAHEAD
      uint8_t *buffer = dev->d_appdata;
      int      buflen = dev->d_len;
      uint16_t recvlen;
#endif

      ninfo("No listener on connection\n");

#ifdef CONFIG_NET_TCP_READAHEAD
      /* Save as the packet data as in the read-ahead buffer.  NOTE that
       * partial packets will not be buffered.
       */

      recvlen = tcp_datahandler(conn, buffer, buflen);
      if (recvlen < buflen)
#endif
        {
          /* There is no handler to receive new data and there are no free
           * read-ahead buffers to retain the data -- drop the packet.
           */

         ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
          g_netstats.tcp.drop++;
#endif
          /* Clear the TCP_SNDACK bit so that no ACK will be sent */

          ret &= ~TCP_SNDACK;
        }
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  /* Preserve the TCP_ACKDATA, TCP_CLOSE, and TCP_ABORT in the response.
   * These is needed by the network to handle responses and buffer state.  The
   * TCP_NEWDATA indication will trigger the ACK response, but must be
   * explicitly set in the callback.
   */

  ninfo("flags: %04x\n", flags);

  /* Perform the data callback.  When a data callback is executed from 'list',
   * the input flags are normally returned, however, the implementation
   * may set one of the following:
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
   *                 to indicate that an ACK should be included in the response.
   *                 (In TCP_NEWDATA is cleared but TCP_SNDACK is not set, then
   *                 dev->d_len should also be cleared).
   */

  flags = devif_conn_event(dev, conn, flags, conn->list);

  /* There may be no new data handler in place at them moment that the new
   * incoming data is received.  If the new incoming data was not handled, then
   * either (1) put the unhandled incoming data in the read-ahead buffer (if
   * enabled) or (2) suppress the ACK to the data in the hope that it will
   * be re-transmitted at a better time.
   */

  if ((flags & TCP_NEWDATA) != 0)
    {
      /* Data was not handled.. dispose of it appropriately */

      flags = tcp_data_event(dev, conn, flags);
    }

  /* Check if there is a connection-related event and a connection
   * callback.
   */

  if ((flags & TCP_CONN_EVENTS) != 0)
    {
      /* Perform the callback disconnect callbacks */

      flags = devif_conn_event(dev, conn, flags, conn->connevents);
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

#ifdef CONFIG_NET_TCP_READAHEAD
uint16_t tcp_datahandler(FAR struct tcp_conn_s *conn, FAR uint8_t *buffer,
                         uint16_t buflen)
{
  FAR struct iob_s *iob;
  int ret;

  /* Try to allocate on I/O buffer to start the chain without waiting (and
   * throttling as necessary).  If we would have to wait, then drop the
   * packet.
   */

  iob = iob_tryalloc(true);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to create new I/O buffer chain\n");
      return 0;
    }

  /* Copy the new appdata into the I/O buffer chain (without waiting) */

  ret = iob_trycopyin(iob, buffer, buflen, 0, true);
  if (ret < 0)
    {
      /* On a failure, iob_copyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  ninfo("Buffered %d bytes\n", buflen);
  return buflen;
}
#endif /* CONFIG_NET_TCP_READAHEAD */

#endif /* NET_TCP_HAVE_STACK */
