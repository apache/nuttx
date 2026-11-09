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
 * Name: can_nextconn()
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_nextconn(FAR struct can_conn_s *conn)
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
  while ((conn = can_nextconn(conn)) != NULL)
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

#endif /* CONFIG_NET_CAN */
