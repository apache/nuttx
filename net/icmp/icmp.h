/****************************************************************************
 * net/icmp/icmp.h
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

#ifndef __NET_ICMP_ICMP_H
#define __NET_ICMP_ICMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_NO_STACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NET_ICMP_HAVE_STACK 1

/* Allocate/free an ICMP data callback */

#define icmp_callback_alloc(dev, conn) \
  devif_callback_alloc((dev), &(conn)->list, &(conn)->list_tail)
#define icmp_callback_free(dev, conn, cb) \
  devif_conn_callback_free((dev), (cb), &(conn)->list, &(conn)->list_tail)

/****************************************************************************
 * Public types
 ****************************************************************************/

struct socket;    /* Forward reference */
struct sockaddr;  /* Forward reference */
struct pollfd;    /* Forward reference */

#ifdef CONFIG_NET_ICMP_SOCKET
/* Representation of a IPPROTO_ICMP socket connection */

struct devif_callback_s;         /* Forward reference */

/* This is a container that holds the poll-related information */

struct icmp_poll_s
{
  FAR struct socket *psock;        /* IPPROTO_ICMP socket structure */
  FAR struct pollfd *fds;          /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

struct icmp_conn_s
{
  /* Common prologue of all connection structures. */

  dq_entry_t node;               /* Supports a doubly linked list */

  /* This is a list of ICMP callbacks.  Each callback represents a thread
   * that is stalled, waiting for a device-specific event.
   */

  FAR struct devif_callback_s *list;
  FAR struct devif_callback_s *list_tail;

  /* ICMP-specific content follows */

  uint16_t   id;                 /* ICMP ECHO request ID */
  uint8_t    nreqs;              /* Number of requests with no response received */
  uint8_t    crefs;              /* Reference counts on this instance */

  /* The device that the ICMP request was sent on */

  FAR struct net_driver_s *dev;  /* Needed to free the callback structure */

  /* ICMP response read-ahead list.  A singly linked list of type struct
   * iob_qentry_s where the ICMP read-ahead data for the current ID is
   * retained.
   */

  struct iob_queue_s readahead;  /* Read-ahead buffering */

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct icmp_poll_s pollinfo[CONFIG_NET_ICMP_NPOLLWAITERS];
};
#endif

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

#ifdef CONFIG_NET_ICMP_SOCKET
/* PF_INET socket address family, IPPROTO_ICMP protocol interface */

EXTERN const struct sock_intf_s g_icmp_sockif;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_input
 *
 * Description:
 *   Handle incoming ICMP input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received ICMP
 *         packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmp_input(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: icmp_sock_initialize
 *
 * Description:
 *   Initialize the IPPROTO_ICMP socket connection structures.  Called once
 *   and only from the network initialization layer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_sock_initialize(void);
#endif

/****************************************************************************
 * Name: icmp_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_alloc(void);
#endif

/****************************************************************************
 * Name: icmp_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_free(FAR struct icmp_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmp_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_active(uint16_t id);
#endif

/****************************************************************************
 * Name: icmp_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_nextconn(FAR struct icmp_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmp_findconn
 *
 * Description:
 *   Find an ICMP connection structure that is expecting a ICMP ECHO response
 *   with this ID from this device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_findconn(FAR struct net_driver_s *dev,
                                      uint16_t id);
#endif

/****************************************************************************
 * Name: icmp_poll
 *
 * Description:
 *   Poll a device "connection" structure for availability of ICMP TX data
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - A pointer to the ICMP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_poll(FAR struct net_driver_s *dev, FAR struct icmp_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmp_sendmsg
 *
 * Description:
 *   Implements the sendmsg() operation for the case of the IPPROTO_ICMP
 *   socket.  The 'buf' parameter points to a block of memory that includes
 *   an ICMP request header, followed by any payload that accompanies the
 *   request.  The 'len' parameter includes both the size of the ICMP header
 *   and the following payload.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Message to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
ssize_t icmp_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                     int flags);
#endif

/****************************************************************************
 * Name: icmp_recvmsg
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   data gram socket with the IPPROTO_ICMP protocol.  icmp_recvmsg()
 *   receives ICMP ECHO replies for the a socket.
 *
 *   If msg_name is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen' is
 *   initialized to the size of the buffer associated with msg_name, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
ssize_t icmp_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                     int flags);
#endif

/****************************************************************************
 * Name: icmp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
int icmp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Name: icmp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
int icmp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Name: icmp_reply
 *
 * Description:
 *   Send an ICMP message in response to a situation
 *   RFC 1122: 3.2.2 MUST send at least the IP header and 8 bytes of header.
 *       MAY send more (we do).
 *       MUST NOT change this header information.
 *       MUST NOT reply to a multicast/broadcast IP address.
 *       MUST NOT reply to a multicast/broadcast MAC address.
 *       MUST reply to only the first fragment.
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received packet
 *   type  - ICMP Message Type, eg. ICMP_DEST_UNREACHABLE
 *   code  - ICMP Message Code, eg. ICMP_PORT_UNREACH
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmp_reply(FAR struct net_driver_s *dev, int type, int code);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ICMP && !CONFIG_NET_ICMP_NO_STACK */
#endif /* __NET_ICMP_ICMP_H */
