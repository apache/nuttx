/****************************************************************************
 * net/can/can.c
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

#ifdef CONFIG_NET_CAN

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>

#include <arch/irq.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/mutex.h>
#include <nuttx/net/can.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>

#include "can/can.h"
#include "devif/devif.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_CAN_MAX_CONNS
#  define CONFIG_NET_CAN_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all NetLink connections. */

NET_BUFPOOL_DECLARE(g_can_connections, sizeof(struct can_conn_s),
                    CONFIG_NET_CAN_PREALLOC_CONNS,
                    CONFIG_NET_CAN_ALLOC_CONNS,
                    CONFIG_NET_CAN_MAX_CONNS);

/* A list of all allocated NetLink connections */

static dq_queue_t g_active_can_connections;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uint8_t g_can_dlc_to_len[16] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  12,
  16,
  20,
  24,
  32,
  48,
  64,
};
const uint8_t g_len_to_can_dlc[65] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  9,
  9,
  9,
  10,
  10,
  10,
  10,
  11,
  11,
  11,
  11,
  12,
  12,
  12,
  12,
  13,
  13,
  13,
  13,
  13,
  13,
  13,
  13,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  14,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
  15,
};

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

static inline uint32_t
can_data_event(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn,
               uint32_t flags)
{
  int buflen = dev->d_len;
  int recvlen;
  uint32_t ret;

  ret = (flags & ~CAN_NEWDATA);

  /* Save as the packet data as in the read-ahead buffer.  NOTE that
   * partial packets will not be buffered.
   */

  recvlen = can_datahandler(dev, conn);
  if (recvlen < buflen)
    {
      /* There is no handler to receive new data and there are no free
       * read-ahead buffers to retain the data -- drop the packet.
       */

      ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.can.drop++;
#endif
      NETDEV_RXDROPPED(dev);
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Name: can_input_conn
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev  - The device driver structure containing the received packet
 *   conn - A pointer to the CAN connection structure
 *
 * Returned Value:
 *   OK     The packet has been processed  and can be deleted
 *  -EAGAIN There is a matching connection, but could not dispatch the packet
 *          yet.  Useful when a packet arrives before a recv call is in
 *          place.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

static int can_input_conn(FAR struct net_driver_s *dev,
                          FAR struct can_conn_s *conn)
{
  uint32_t flags;
  uint16_t buflen = dev->d_len;
  int ret = OK;

  /* Setup for the application callback */

  dev->d_appdata = dev->d_buf;
  dev->d_sndlen  = 0;
  dev->d_len     = buflen;

  /* Perform the application callback */

  flags = can_callback(dev, conn, CAN_NEWDATA);

  /* If the operation was successful, the CAN_NEWDATA flag is removed
   * and thus the packet can be deleted (OK will be returned).
   */

  if ((flags & CAN_NEWDATA) != 0)
    {
      /* No.. the packet was not processed now.  Return -EAGAIN so
       * that the driver may retry again later.  We still need to
       * set d_len to zero so that the driver is aware that there
       * is nothing to be sent.
       */

      nwarn("WARNING: Packet not processed\n");
      ret = -EAGAIN;
    }

  return ret;
}

/****************************************************************************
 * Name: can_in
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK     The packet has been processed  and can be deleted
 *  -EAGAIN There is a matching connection, but could not dispatch the packet
 *          yet.  Useful when a packet arrives before a recv call is in
 *          place.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

static int can_in(FAR struct net_driver_s *dev)
{
  FAR struct can_conn_s *conn = can_active(dev, NULL);
  FAR struct can_conn_s *nextconn;
  int ret;

  if (conn == NULL)
    {
      /* There is no listener on the dev.  Just drop the packet. */

      return OK;
    }

  /* Storing reception timestamp provided by realtime
   * if timestamp no provided by hardware.
   */

#ifdef CONFIG_NET_TIMESTAMP
  if ((dev->d_features & NETDEV_RX_STAMP) == 0)
    {
      clock_gettime(CLOCK_REALTIME, &dev->d_iob->io_time);
    }
#endif /* CONFIG_NET_TIMESTAMP */

  can_conn_list_lock();

  /* Do we have second connection that can hold this packet? */

  while ((nextconn = can_active(dev, conn)) != NULL)
    {
      /* Yes... There are multiple listeners on the same dev.
       * We need to clone the packet and deliver it to each listener.
       */

      FAR struct iob_s *iob = can_iob_clone(dev);

      if (iob == NULL)
        {
          nerr("ERROR: IOB clone failed.\n");
          break; /* We can still process one time without clone. */
        }

      can_input_conn(dev, conn);

      netdev_iob_replace(dev, iob);
      conn = nextconn;
    }

  /* We can deliver the packet directly to the last listener. */

  ret = can_input_conn(dev, conn);

  can_conn_list_unlock();

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

uint32_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint32_t flags)
{
  /* Some sanity checking */

  if (conn)
    {
      conn_lock(&conn->sconn);
      flags = devif_conn_event(dev, flags, conn->sconn.list);

      /* Either we did not get the lock or there is no application listening
       * If we did not get a lock we store the frame in the read-ahead buffer
       */

      if ((flags & CAN_NEWDATA) != 0)
        {
          /* Data was not handled.. dispose of it appropriately */

          flags = can_data_event(dev, conn, flags);
        }

      conn_unlock(&conn->sconn);
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
 *   dev  - The device which as active when the event was detected.
 *   conn - A pointer to the CAN connection structure
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

int can_datahandler(FAR struct net_driver_s *dev,
                    FAR struct can_conn_s *conn)
{
  FAR struct iob_s *iob = dev->d_iob;
  int ret = 0;

  conn_lock(&conn->sconn);
#if CONFIG_NET_RECV_BUFSIZE > 0
#  if CONFIG_NET_CAN_NBUFFERS > 0
  int bufnum = div_const_roundup(conn->rcvbufs, NET_CAN_PKTSIZE);
#  else
  int bufnum = div_const_roundup(conn->rcvbufs, CONFIG_IOB_BUFSIZE);
#  endif
  /* Check the frame count pending on conn->readahead */

  if (iob_get_queue_entry_count(&conn->readahead) >= bufnum)
    {
      nwarn("WARNING: There are no free receive buffer to retain the data. "
            "Receive buffer number:%d, received frames:%u \n",
            bufnum, iob_get_queue_entry_count(&conn->readahead));
      goto errout;
    }
#endif

  /* Concat the iob to readahead */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_CAN_NOTIFIER
      /* Provide notification(s) that additional CAN read-ahead data is
       * available.
       */

      can_readahead_signal(conn);
#endif
      ret = iob->io_pktlen;

      /* Device buffer has been enqueued, clear the handle */

      netdev_iob_clear(dev);
    }
  else
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      ret = 0;
      goto errout;
    }

  conn_unlock(&conn->sconn);
  return ret;

errout:
  netdev_iob_release(dev);
  conn_unlock(&conn->sconn);
  return ret;
}

/****************************************************************************
 * Name: can_input
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK     The packet has been processed  and can be deleted
 *  -EAGAIN There is a matching connection, but could not dispatch the packet
 *          yet.  Useful when a packet arrives before a recv call is in
 *          place.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

int can_input(FAR struct net_driver_s *dev)
{
  FAR uint8_t *buf;
  int ret;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.can.recv++;
#endif
  netdev_lock(dev);

  if (dev->d_iob != NULL)
    {
      buf = dev->d_buf;

      /* Set the device buffer to l2 */

      dev->d_buf = NETLLBUF;
      ret = can_in(dev);

      dev->d_buf = buf;

      netdev_unlock(dev);
      return ret;
    }

  ret = netdev_input(dev, can_in, false);
  if (ret < 0)
    {
#ifdef CONFIG_NET_STATISTICS
      g_netstats.can.drop++;
#endif
    }

  netdev_unlock(dev);
  return ret;
}

/****************************************************************************
 * Name: can_poll
 *
 * Description:
 *   Poll a packet "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The packet "connection" to poll for TX data
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

void can_poll(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn)
{
  DEBUGASSERT(dev != NULL && conn != NULL && dev == conn->dev);

  /* Setup for the application callback */

  dev->d_len    = 0;
  dev->d_sndlen = 0;

  /* Perform the application callback */

  can_callback(dev, conn, CAN_POLL);

  /* Check if the application has data to send */

  if (dev->d_sndlen > 0)
    {
      return;
    }

  /* Make sure that d_len is zero meaning that there is nothing to be sent */

  dev->d_len = 0;
}

#ifdef CONFIG_NET_CAN_NOTIFIER
/****************************************************************************
 * Name: can_readahead_signal
 *
 * Description:
 *   Read-ahead data has been buffered.  Signal all threads waiting for
 *   read-ahead data to become available.
 *
 *   When read-ahead data becomes available, *all* of the workers waiting
 *   for read-ahead data will be executed.  If there are multiple workers
 *   waiting for read-ahead data then only the first to execute will get the
 *   data.  Others will need to call can_readahead_notifier_setup() once
 *   again.
 *
 * Input Parameters:
 *   conn  - The CAN connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void can_readahead_signal(FAR struct can_conn_s *conn)
{
  /* This is just a simple wrapper around work_notifier_signal(). */

  work_notifier_signal(WORK_CAN_READAHEAD, conn);
}
#endif

/****************************************************************************
 * Name: can_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct can_conn_s *can_alloc(void)
{
  FAR struct can_conn_s *conn;

  /* The free list is protected by a a mutex. */

  NET_BUFPOOL_LOCK(g_can_connections);

  conn = NET_BUFPOOL_TRYALLOC(g_can_connections);
  if (conn != NULL)
    {
      /* FIXME SocketCAN default behavior enables loopback */

#ifdef CONFIG_NET_CANPROTO_OPTIONS
      /* By default the filter is configured to catch all,
       * this is done in commented filter code below:
       *
       * struct can_filter_t catchall_filter;
       * filter.can_id = 0;
       * filter.can_mask = 0;
       * conn->filters[0] = catchall_filter;
       *
       * However memset already sets the filter to 0
       * therefore we only have to set the filter count to 1
       */

      conn->filter_count = 1;
#endif

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_can_connections);
    }

  NET_BUFPOOL_UNLOCK(g_can_connections);
  return conn;
}

/****************************************************************************
 * Name: can_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void can_free(FAR struct can_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  NET_BUFPOOL_LOCK(g_can_connections);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_can_connections);
  nxrmutex_destroy(&conn->sconn.s_lock);

#ifdef CONFIG_NET_CAN_WRITE_BUFFERS
  /* Free the write queue */

  iob_free_queue(&conn->write_q);

#   if CONFIG_NET_SEND_BUFSIZE > 0
  /* Notify the send buffer available */

  can_sendbuffer_notify(conn);
#   endif /* CONFIG_NET_SEND_BUFSIZE */

#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
  nxsem_destroy(&conn->sndsem);
#endif

  /* Free the readahead queue */

  iob_free_queue(&conn->readahead);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_can_connections, conn);

  NET_BUFPOOL_UNLOCK(g_can_connections);
}

/****************************************************************************
 * Name: can_recv_filter
 *
 * Description:
 *   filter incoming packet
 *
 * Input Parameters:
 *   conn - A pointer to the CAN connection structure
 *   id   - The CAN identifier
 *
 * Returned Value: 0 - Filter not passed, 1 - Filter passed
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CANPROTO_OPTIONS
static int can_recv_filter(FAR struct can_conn_s *conn, canid_t id)
{
  uint32_t i;

#ifdef CONFIG_NET_CAN_ERRORS
  /* error message frame */

  if ((id & CAN_ERR_FLAG) != 0)
    {
      return id & conn->err_mask ? 1 : 0;
    }
#endif

  for (i = 0; i < conn->filter_count; i++)
    {
      if (conn->filters[i].can_id & CAN_INV_FILTER)
        {
          if ((id & conn->filters[i].can_mask) !=
                ((conn->filters[i].can_id & ~CAN_INV_FILTER) &
                 conn->filters[i].can_mask))
            {
              return 1;
            }
        }
      else
        {
          if ((id & conn->filters[i].can_mask) ==
                (conn->filters[i].can_id & conn->filters[i].can_mask))
            {
              return 1;
            }
        }
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: can_conn_next
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_conn_next(FAR struct can_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct can_conn_s *)g_active_can_connections.head;
    }
  else
    {
      return (FAR struct can_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: can_conn_list_lock
 *       can_conn_list_unlock
 *
 * Description:
 *   Lock and unlock the CAN connection list. This is used to protect
 *   the list of active connections.
 *
 * Assumptions:
 *   This function is called from driver.
 *
 ****************************************************************************/

void can_conn_list_lock(void)
{
  NET_BUFPOOL_LOCK(g_can_connections);
}

void can_conn_list_unlock(void)
{
  NET_BUFPOOL_UNLOCK(g_can_connections);
}

/****************************************************************************
 * Name: can_active()
 *
 * Description:
 *   Traverse the list of NetLink connections that match dev
 *
 * Input Parameters:
 *   dev  - The device to search for.
 *   conn - The current connection; may be NULL to start the search at the
 *          beginning
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_active(FAR struct net_driver_s *dev,
                                  FAR struct can_conn_s *conn)
{
#ifdef CONFIG_NET_CANPROTO_OPTIONS
  canid_t can_id;

  memcpy(&can_id, NETLLBUF, sizeof(canid_t));
#endif

  can_conn_list_lock();
  while ((conn = can_conn_next(conn)) != NULL)
    {
      if ((conn->dev == NULL && _SS_ISBOUND(conn->sconn.s_flags)) ||
          conn->dev == dev)
        {
          if (dev->d_iob->io_conn == &conn->sconn
#ifdef CONFIG_NET_CANPROTO_OPTIONS
              && !_SO_GETOPT(conn->sconn.s_options, CAN_RAW_RECV_OWN_MSGS)
#endif
             )
            {
              continue; /* Skip own messages */
            }

#ifdef CONFIG_NET_CANPROTO_OPTIONS
          if (can_recv_filter(conn, can_id) == 0)
            {
              continue;
            }
#endif

          /* Do not pass CAN FD frames to a classic CAN socket */

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
          if (!_SO_GETOPT(conn->sconn.s_options, CAN_RAW_FD_FRAMES))
#endif
            {
              if (dev->d_len > sizeof(struct can_frame))
                {
                  continue;
                }
            }

          break;
        }
    }

  can_conn_list_unlock();
  return conn;
}

/****************************************************************************
 * Name: can_get_ifindex
 *
 * Description:
 *   Return the interface index of the device a CAN connection is bound to,
 *   or 0 if it is not bound to a device.
 *
 * Input Parameters:
 *   conn - The CAN connection
 *
 ****************************************************************************/

int can_get_ifindex(FAR struct can_conn_s *conn)
{
  if (conn->dev != NULL)
    {
#ifdef CONFIG_NETDEV_IFINDEX
      return conn->dev->d_ifindex;
#else
      /* Reverse mapping: extract ifindex from device name "can<N>".
       * This mirrors the bind logic which maps ifindex -> "can<N>".
       */

      return conn->dev->d_ifname[3] - '0';
#endif
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Name: can_conn_bind
 *
 * Description:
 *   Bind a CAN connection to the network device selected by the
 *   can_ifindex field of a struct sockaddr_can.
 *
 * Input Parameters:
 *   conn    - The connection to bind
 *   addr    - The struct sockaddr_can address
 *   addrlen - The length of addr
 *
 * Returned Value:
 *   OK on success, -ENODEV if a non-zero interface index does not match a
 *   CAN device.
 *
 ****************************************************************************/

int can_conn_bind(FAR struct can_conn_s *conn,
                  FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct sockaddr_can *canaddr;

  DEBUGASSERT(addr != NULL &&
              addrlen >= sizeof(struct sockaddr_can));

  /* Save the address information in the connection structure */

  canaddr = (FAR struct sockaddr_can *)addr;

  /* Bind CAN device to socket */

#ifdef CONFIG_NETDEV_IFINDEX
  conn->dev = netdev_findbyindex(canaddr->can_ifindex);
#else
  char netdev_name[5] = "can0";

  netdev_name[3] += canaddr->can_ifindex;
  conn->dev = netdev_findbyname((const char *)&netdev_name);
#endif

  return conn->dev == NULL && canaddr->can_ifindex != 0 ? -ENODEV : OK;
}

#endif /* CONFIG_NET_CAN */
