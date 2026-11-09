/****************************************************************************
 * net/can/can.h
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

#ifndef __NET_CAN_CAN_H
#define __NET_CAN_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <poll.h>

#include <nuttx/semaphore.h>
#include <nuttx/can.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_CAN_NOTIFIER
#  include <nuttx/wqueue.h>
#endif

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new packet socket data callback */

#define can_callback_alloc(dev,conn) \
  devif_callback_alloc(dev, &conn->sconn.list, &conn->sconn.list_tail)
#define can_callback_free(dev,conn,cb) \
  devif_conn_callback_free(dev, cb, &conn->sconn.list, &conn->sconn.list_tail)

#ifndef CONFIG_NET_CAN_NBUFFERS
#  define CONFIG_NET_CAN_NBUFFERS 0
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This is a container that holds the poll-related information */

struct can_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  FAR struct net_driver_s *dev;    /* Needed to free the callback structure */
  FAR struct pollfd_s *fds;        /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

/* This "connection" structure describes the underlying state of the socket */

struct can_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  FAR struct net_driver_s *dev;      /* Reference to CAN device */

  /* Read-ahead buffering.
   *
   *   readahead - A singly linked list of type struct iob_qentry_s
   *               where the CAN/IP read-ahead data is retained.
   */

  struct iob_queue_s readahead;      /* remove Read-ahead buffering */

#if CONFIG_NET_RECV_BUFSIZE > 0
  int32_t  rcvbufs;                  /* Maximum amount of bytes queued in receive */
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
  int32_t  sndbufs;                  /* Maximum amount of bytes queued in send */
  sem_t    sndsem;                   /* Semaphore signals send completion */
#endif

#ifdef CONFIG_NET_CAN_WRITE_BUFFERS
  /* Write buffering
   *
   *   write_q   - The queue of unsent I/O buffers.  The head of this
   *               list may be partially sent.  FIFO ordering.
   */

  struct iob_queue_s write_q;        /* Write buffering for can messages */

  /* Callback instance for can send */

  FAR struct devif_callback_s *sndcb;
#endif

  /* CAN-specific content follows */

  int16_t crefs;                     /* Reference count */

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct can_poll_s pollinfo[CONFIG_NET_CAN_NPOLLWAITERS];

#ifdef CONFIG_NET_CANPROTO_OPTIONS
  struct can_filter filters[CONFIG_NET_CAN_RAW_FILTER_MAX];
  int32_t filter_count;
#  ifdef CONFIG_NET_CAN_ERRORS
  can_err_mask_t err_mask;
#  endif
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern const struct sock_intf_s g_can_raw_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: can_sockif
 *
 * Description:
 *   Return the socket interface of the requested CAN protocol.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
can_sockif(sa_family_t family, int type, int protocol);

/****************************************************************************
 * Name: can_conn_init
 *
 * Description:
 *   Initialize the state shared by all CAN protocols in a newly allocated
 *   connection and add it to the list of active connections.
 *
 ****************************************************************************/

void can_conn_init(FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_conn_uninit
 *
 * Description:
 *   Remove a connection from the list of active connections and release
 *   the state shared by all CAN protocols.  The caller frees the
 *   connection itself.
 *
 ****************************************************************************/

void can_conn_uninit(FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_conn_bind
 *
 * Description:
 *   Bind a CAN connection to the network device selected by the
 *   can_ifindex field of a struct sockaddr_can.
 *
 ****************************************************************************/

int can_conn_bind(FAR struct can_conn_s *conn,
                  FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: can_get_ifindex
 *
 * Description:
 *   Return the interface index of the device a CAN connection is bound to,
 *   or 0 if it is not bound to a device.
 *
 ****************************************************************************/

int can_get_ifindex(FAR struct can_conn_s *conn);

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

FAR struct can_conn_s *can_conn_next(FAR struct can_conn_s *conn);

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

void can_conn_list_lock(void);
void can_conn_list_unlock(void);

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
                                  FAR struct can_conn_s *conn);

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
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

uint32_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint32_t flags);

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
 *   The number of bytes actually buffered is returned.  This will be
 *   negative or zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that CAN_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

int can_datahandler(FAR struct net_driver_s *dev,
                    FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_poll
 *
 * Description:
 *   Poll a CAN connection structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The CAN "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void can_poll(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn);

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

#ifdef CONFIG_NET_CAN_NOTIFIER
void can_readahead_signal(FAR struct can_conn_s *conn);
#endif

/****************************************************************************
 * Name: can_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The CAN connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
void can_sendbuffer_notify(FAR struct can_conn_s *conn);
#endif /* CONFIG_NET_SEND_BUFSIZE */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_CAN */
#endif /* __NET_CAN_CAN_H */
