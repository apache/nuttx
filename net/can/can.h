/****************************************************************************
 * net/can/can.h
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

#include <netpacket/can.h>
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

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This is a container that holds the poll-related information */

struct can_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  FAR struct net_driver_s *dev;    /* Needed to free the callback structure */
  struct pollfd *fds;              /* Needed to handle poll events */
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

  /* CAN-specific content follows */

  uint8_t protocol;                  /* Selected CAN protocol */
  int16_t crefs;                     /* Reference count */

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct can_poll_s pollinfo[4]; /* FIXME make dynamic */

#ifdef CONFIG_NET_CANPROTO_OPTIONS
  int32_t loopback;
  int32_t recv_own_msgs;
#ifdef CONFIG_NET_CAN_CANFD
  int32_t fd_frames;
#endif
  struct can_filter filters[CONFIG_NET_CAN_RAW_FILTER_MAX];
  int32_t filter_count;
# ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t tx_deadline;
# endif
#endif
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

EXTERN const struct sock_intf_s g_can_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct sockaddr_can;  /* Forward reference */

/****************************************************************************
 * Name: can_initialize()
 *
 * Description:
 *   Initialize the NetLink connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void can_initialize(void);

/****************************************************************************
 * Name: can_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct can_conn_s *can_alloc(void);

/****************************************************************************
 * Name: can_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void can_free(FAR struct can_conn_s *conn);

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

FAR struct can_conn_s *can_nextconn(FAR struct can_conn_s *conn);

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

uint16_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint16_t flags);

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
 * - Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint16_t can_datahandler(FAR struct can_conn_s *conn, FAR uint8_t *buffer,
                         uint16_t buflen);

/****************************************************************************
 * Name: can_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen'
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored)
 *
 ****************************************************************************/

ssize_t can_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags);

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
 * Name: can_sendmsg
 *
 * Description:
 *   The can_sendmsg() call may be used only when the packet socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      CAN frame and optional CMSG
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendmsg() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t can_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags);

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
 * Name: can_setsockopt
 *
 * Description:
 *   can_setsockopt() sets the CAN-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/can.h> for the a complete list of values of CAN protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CANPROTO_OPTIONS
int can_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Name: can_getsockopt
 *
 * Description:
 *   can_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netpacket/can.h for the case of the CAN protocol).
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CANPROTO_OPTIONS
int can_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_CAN */
#endif /* __NET_CAN_CAN_H */
