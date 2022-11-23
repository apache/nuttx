/****************************************************************************
 * net/can/can_callback.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/mm/iob.h>

#include "devif/devif.h"
#include "can/can.h"

#ifdef CONFIG_NET_TIMESTAMP
#include <sys/time.h>
#endif

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
  FAR uint8_t *buffer = dev->d_appdata;
  int buflen = dev->d_len;
  uint16_t recvlen;

  ret = (flags & ~CAN_NEWDATA);

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
      /* No support CAN net statistics yet */

      /* g_netstats.tcp.drop++; */

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
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

uint16_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint16_t flags)
{
  /* Some sanity checking */

  if (conn)
    {
#ifdef CONFIG_NET_TIMESTAMP
      /* TIMESTAMP sockopt is activated, create timestamp and copy to iob */

      if (conn->sconn.s_timestamp)
        {
          struct timespec *ts = (struct timespec *)
                                                &dev->d_appdata[dev->d_len];
          struct timeval *tv = (struct timeval *)
                                                &dev->d_appdata[dev->d_len];
          dev->d_len += sizeof(struct timeval);
          clock_systime_timespec(ts);
          tv->tv_usec = ts->tv_nsec / 1000;
        }
#endif

      /* Try to lock the network when successful send data to the listener */

      if (net_trylock() == OK)
        {
          flags = devif_conn_event(dev, flags, conn->sconn.list);
          net_unlock();
        }

      /* Either we did not get the lock or there is no application listening
       * If we did not get a lock we store the frame in the read-ahead buffer
       */

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
      iob_free_chain(iob);
      return 0;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
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
