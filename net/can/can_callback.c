/****************************************************************************
 * net/pkt/pkt_callback.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "can/can.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_data_event
 *
 * Description:
 *   Handle data that is not accepted by the application because there is no
 *   listener in place ready to receive the data.
 *
 * Assumptions:
 * - The caller has checked that CAN_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

static inline uint16_t
can_data_event(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn,
               uint16_t flags)
{
  uint16_t ret;
  uint8_t *buffer = dev->d_appdata;
  int      buflen = dev->d_len;
  uint16_t recvlen;

  ret = (flags & ~CAN_NEWDATA);

  //ninfo("No listener on connection\n");

   /* Save as the packet data as in the read-ahead buffer.  NOTE that
    * partial packets will not be buffered.
    */

  recvlen = can_datahandler(conn, buffer, buflen);
  if (recvlen < buflen)
    {
      /* There is no handler to receive new data and there are no free
       * read-ahead buffers to retain the data -- drop the packet.
       */

      ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
      //g_netstats.tcp.drop++;
#endif
    }
        
  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_callback
 *
 * Description:
 *   Inform the application holding the packet socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

uint16_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint16_t flags)
{
  /* Some sanity checking */

  if (conn)
    {
      /* Perform the callback */

      flags = devif_conn_event(dev, conn, flags, conn->list);
    
    if ((flags & CAN_NEWDATA) != 0)
      {
        /* Data was not handled.. dispose of it appropriately */

        flags = can_data_event(dev, conn, flags);
      }
  }
  
  return flags;
}

/****************************************************************************
 * Name: can_datahandler
 *
 * Description:
 *   Handle data that is not accepted by the application.  This may be called
 *   either (1) from the data receive logic if it cannot buffer the data, or
 *   (2) from the CAN event logic is there is no listener in place ready to
 *   receive the data.
 *
 * Input Parameters:
 *   conn - A pointer to the CAN connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that CAN_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t can_datahandler(FAR struct can_conn_s *conn, FAR uint8_t *buffer,
                         uint16_t buflen)
{
  FAR struct iob_s *iob;
  int ret;

  /* Try to allocate on I/O buffer to start the chain without waiting (and
   * throttling as necessary).  If we would have to wait, then drop the
   * packet.
   */

  iob = iob_tryalloc(true, IOBUSER_NET_CAN_READAHEAD);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to create new I/O buffer chain\n");
      return 0;
    }

  /* Copy the new appdata into the I/O buffer chain (without waiting) */

  ret = iob_trycopyin(iob, buffer, buflen, 0, true,
                      IOBUSER_NET_CAN_READAHEAD);
  if (ret < 0)
    {
      /* On a failure, iob_copyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob, IOBUSER_NET_CAN_READAHEAD);
      return 0;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob, IOBUSER_NET_TCP_READAHEAD);
      return 0;
    }
    
#ifdef CONFIG_NET_CAN_NOTIFIER
  /* Provide notification(s) that additional CAN read-ahead data is
   * available.
   */

  can_readahead_signal(conn);
#endif
  return buflen;
}

#endif /* CONFIG_NET && CONFIG_NET_CAN */
