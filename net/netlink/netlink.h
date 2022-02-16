/****************************************************************************
 * net/netlink/netlink.h
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

#ifndef __NET_NETLINK_NETLINK_H
#define __NET_NETLINK_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>
#include <poll.h>

#include <netpacket/netlink.h>
#include <nuttx/net/netlink.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "devif/devif.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NETLINK_ROUTE
  #define netlink_device_notify(dev)
#endif

#ifdef CONFIG_NET_NETLINK

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This connection structure describes the underlying state of the socket. */

struct netlink_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  /* NetLink-specific content follows */

  uint32_t pid;                      /* Port ID (if bound) */
  uint32_t groups;                   /* Multicast groups mask (if bound) */
  uint32_t dst_pid;                  /* Destination port ID */
  uint32_t dst_groups;               /* Destination multicast groups mask */
  uint8_t crefs;                     /* Reference counts on this instance */
  uint8_t protocol;                  /* See NETLINK_* definitions */

  /* poll() support */

  int key;                           /* used to cancel notifications */
  FAR sem_t *pollsem;                /* Used to wakeup poll() */
  FAR pollevent_t *pollevent;        /* poll() wakeup event */

  /* Queued response data */

  sq_queue_t resplist;               /* Singly linked list of responses */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

EXTERN const struct sock_intf_s g_netlink_sockif;

/****************************************************************************
 * Name: netlink_initialize()
 *
 * Description:
 *   Initialize the NetLink connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void netlink_initialize(void);

/****************************************************************************
 * Name: netlink_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_alloc(void);

/****************************************************************************
 * Name: netlink_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_nextconn()
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_nextconn(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function the Netlink
 *   response data is received.  The worker function will execute on the low
 *   priority worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the low priority work
 *            queue when Netlink response data is available.
 *   conn   - The Netlink connection where the response is expected.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   Zero (OK) is returned if the notification was successfully set up.
 *   A negated error value is returned if an unexpected error occurred
 *   and no notification will occur.
 *
 ****************************************************************************/

int netlink_notifier_setup(worker_t worker, FAR struct netlink_conn_s *conn,
                           FAR void *arg);

/****************************************************************************
 * Name: netlink_notifier_teardown
 *
 * Description:
 *   Eliminate a Netlink response notification previously setup by
 *   netlink_notifier_setup().  This function should only be called if the
 *   notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the notification.
 *
 * Input Parameters:
 *   conn - Teardown the notification for this Netlink connection.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int netlink_notifier_teardown(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_notifier_signal
 *
 * Description:
 *   New Netlink response data is available.  Execute worker thread
 *   functions for all threads that wait for response data.
 *
 * Input Parameters:
 *   conn - The Netlink connection where the response was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void netlink_notifier_signal(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_tryget_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  NULL will be returned if the pending response list is
 *   empty
 *
 ****************************************************************************/

FAR struct netlink_response_s *
netlink_tryget_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_get_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  This function will block until a response is received if
 *   the pending response list is empty.  NULL will be returned only in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct netlink_response_s *
netlink_get_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_check_response
 *
 * Description:
 *   Return true is a response is pending now.
 *
 * Returned Value:
 *   True: A response is available; False; No response is available.
 *
 ****************************************************************************/

bool netlink_check_response(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_route_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#ifdef CONFIG_NETLINK_ROUTE
ssize_t netlink_route_sendto(NETLINK_HANDLE handle,
                             FAR const struct nlmsghdr *nlmsg,
                             size_t len, int flags,
                             FAR const struct sockaddr_nl *to,
                             socklen_t tolen);

/****************************************************************************
 * Name: netlink_device_notify()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

void netlink_device_notify(FAR struct net_driver_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_NETLINK */
#endif /* __NET_NETLINK_NETLINK_H */
