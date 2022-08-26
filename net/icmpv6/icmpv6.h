/****************************************************************************
 * net/icmpv6/icmpv6.h
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

#ifndef __NET_ICMPv6_ICMPv6_H
#define __NET_ICMPv6_ICMPv6_H

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
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>

#if defined(CONFIG_NET_ICMPv6) && !defined(CONFIG_NET_ICMPv6_NO_STACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define NET_ICMPv6_HAVE_STACK 1

/* Allocate a new ICMPv6 data callback */

#define icmpv6_callback_alloc(dev, conn) \
  devif_callback_alloc((dev), &(conn)->sconn.list, &(conn)->sconn.list_tail)
#define icmpv6_callback_free(dev, conn, cb) \
  devif_conn_callback_free((dev), (cb), &(conn)->sconn.list, &(conn)->sconn.list_tail)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
struct socket;       /* Forward reference */
struct sockaddr;     /* Forward reference */
struct pollfd;       /* Forward reference */

#ifdef CONFIG_NET_ICMPv6_SOCKET
/* Representation of a IPPROTO_ICMP socket connection */

struct devif_callback_s; /* Forward reference */

/* This is a container that holds the poll-related information */

struct icmpv6_poll_s
{
  FAR struct socket *psock;        /* IPPROTO_ICMP6 socket structure */
  FAR struct pollfd *fds;          /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

struct icmpv6_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  /* ICMPv6-specific content follows */

  uint16_t   id;       /* ICMPv6 ECHO request ID */
  uint8_t    nreqs;    /* Number of requests with no response received */
  uint8_t    crefs;    /* Reference counts on this instance */

  /* The device that the ICMPv6 request was sent on */

  FAR struct net_driver_s *dev;  /* Needed to free the callback structure */

  /* ICMPv6 response read-ahead list.  A singly linked list of type struct
   * iob_qentry_s where the ICMPv6 read-ahead data for the current ID is
   * retained.
   */

  struct iob_queue_s readahead;  /* Read-ahead buffering */

  /* The following is a list of poll structures of threads waiting for
   * socket events.
   */

  struct icmpv6_poll_s pollinfo[CONFIG_NET_ICMPv6_NPOLLWAITERS];
};
#endif

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
/* Used to notify a thread waiting for a particular Neighbor Advertisement */

struct icmpv6_notify_s
{
  FAR struct icmpv6_notify_s *nt_flink;  /* Supports singly linked list */
  net_ipv6addr_t nt_ipaddr;              /* Waited for IP address in the mapping */
  sem_t nt_sem;                          /* Will wake up the waiter */
  int nt_result;                         /* The result of the wait */
};
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
/* Used to notify a thread waiting for a particular Router Advertisement */

struct icmpv6_rnotify_s
{
  FAR struct icmpv6_rnotify_s *rn_flink; /* Supports singly linked list */
  char rn_ifname[IFNAMSIZ];              /* Device name */
  sem_t rn_sem;                          /* Will wake up the waiter */
  int rn_result;                         /* The result of the wait */
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

#ifdef CONFIG_NET_ICMPv6_SOCKET
/* PF_INET6 socket address family, IPPROTO_ICMP6 protocol interface */

EXTERN const struct sock_intf_s g_icmpv6_sockif;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_input
 *
 * Description:
 *   Handle incoming ICMPv6 input
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received ICMPv6
 *           packet
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev, unsigned int iplen);

/****************************************************************************
 * Name: icmpv6_neighbor
 *
 * Description:
 *   The icmpv6_solicit() call may be to send an ICMPv6 Neighbor
 *   Solicitation to resolve an IPv6 address.  This function first checks if
 *   the IPv6 address is already in the Neighbor Table.  If so, then it
 *   returns success immediately.
 *
 *   If the requested IPv6 address in not in the Neighbor Table, then this
 *   function will send the Neighbor Solicitation, delay, then check if the
 *   IP address is now in the Neighbor able.  It will repeat this sequence
 *   until either (1) the IPv6 address mapping is now in the Neighbor table,
 *   or (2) a configurable number of timeouts occur without receiving the
 *   ICMPv6 Neighbor Advertisement.
 *
 * Input Parameters:
 *   ipaddr   The IPv6 address to be queried.
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the Neighbor Table.
 *   On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_neighbor(const net_ipv6addr_t ipaddr);
#else
#  define icmpv6_neighbor(i) (0)
#endif

/****************************************************************************
 * Name: icmpv6_poll
 *
 * Description:
 *   Poll a UDP "connection" structure for availability of ICMPv6 TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ICMPv6_SOCKET) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
void icmpv6_poll(FAR struct net_driver_s *dev,
                 FAR struct icmpv6_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmpv6_solicit
 *
 * Description:
 *   Set up to send an ICMPv6 Neighbor Solicitation message
 *
 * Input Parameters:
 *   dev    - Reference to a device driver structure
 *   ipaddr - IP address of Neighbor to be solicited
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_solicit(FAR struct net_driver_s *dev,
                    FAR const net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name: icmpv6_rsolicit
 *
 * Description:
 *   Set up to send an ICMPv6 Router Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The IPv6 header
 *   - The ICMPv6 Neighbor Router Message
 *
 *   The device IP address should have been set to the link local address
 *   prior to calling this function.
 *
 * Input Parameters:
 *   dev - Reference to a device driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rsolicit(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_advertise
 *
 * Description:
 *   Send an ICMPv6 Neighbor Advertisement
 *
 * Input Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

void icmpv6_advertise(FAR struct net_driver_s *dev,
                      const net_ipv6addr_t destipaddr);

/****************************************************************************
 * Name: icmpv6_radvertise
 *
 * Description:
 *   Send an ICMPv6 Router Advertisement
 *
 * Input Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_ROUTER
void icmpv6_radvertise(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_wait_setup
 *
 * Description:
 *   Called BEFORE an Neighbor Solicitation is sent.  This function sets up
 *   the Neighbor Advertisement timeout before the Neighbor Solicitation
 *   is sent so that there is no race condition when icmpv6_wait() is called.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and executes in the
 *   normal tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
void icmpv6_wait_setup(const net_ipv6addr_t ipaddr,
                       FAR struct icmpv6_notify_s *notify);
#else
#  define icmpv6_wait_setup(i,n)
#endif

/****************************************************************************
 * Name: icmpv6_wait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_wait_setup is called but before
 *   icmpv6_wait()is called (icmpv6_wait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_wait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_wait_cancel(FAR struct icmpv6_notify_s *notify);
#else
#  define icmpv6_wait_cancel(n) (0)
#endif

/****************************************************************************
 * Name: icmpv6_wait
 *
 * Description:
 *   Called each time that a Neighbor Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Neighbor Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and must execute with
 *   the network un-locked (interrupts may be disabled to keep the things
 *   stable).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_wait(FAR struct icmpv6_notify_s *notify, unsigned int timeout);
#else
#  define icmpv6_wait(n,t) (0)
#endif

/****************************************************************************
 * Name: icmpv6_notify
 *
 * Description:
 *   Called each time that a Neighbor Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Neighbor
 *   Advertisement.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
void icmpv6_notify(net_ipv6addr_t ipaddr);
#else
#  define icmpv6_notify(i)
#endif

/****************************************************************************
 * Name: icmpv6_autoconfig
 *
 * Description:
 *   Perform IPv6 auto-configuration to assign an IPv6 address to this
 *   device.
 *
 * Input Parameters:
 *   dev   - The device driver structure to assign the address to
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_autoconfig(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_setaddresses
 *
 * Description:
 *   We successfully obtained the Router Advertisement.  Set the new IPv6
 *   addresses in the driver structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_setaddresses(FAR struct net_driver_s *dev,
                         const net_ipv6addr_t draddr,
                         const net_ipv6addr_t prefix,
                         unsigned int preflen);
#else
#  define icmpv6_setaddresses(dev,draddr,prefix,preflen) (0)
#endif

/****************************************************************************
 * Name: icmpv6_rwait_setup
 *
 * Description:
 *   Called BEFORE an Router Solicitation is sent.  This function sets up
 *   the Router Advertisement timeout before the Router Solicitation
 *   is sent so that there is no race condition when icmpv6_rwait() is
 *   called.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and executes in the
 *   normal tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rwait_setup(FAR struct net_driver_s *dev,
                        FAR struct icmpv6_rnotify_s *notify);
#else
#  define icmpv6_rwait_setup(d,n)
#endif

/****************************************************************************
 * Name: icmpv6_rwait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_rwait_setup() is called but before
 *   icmpv6_rwait()is called (icmpv6_rwait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_rwait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_rwait_cancel(FAR struct icmpv6_rnotify_s *notify);
#else
#  define icmpv6_rwait_cancel(n) (0)
#endif

/****************************************************************************
 * Name: icmpv6_rwait
 *
 * Description:
 *   Called each time that a Router Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Router Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and must execute with
 *   the network un-locked (interrupts may be disabled to keep the things
 *   stable).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_rwait(FAR struct icmpv6_rnotify_s *notify, unsigned int timeout);
#else
#  define icmpv6_rwait(n,t) (0)
#endif

/****************************************************************************
 * Name: icmpv6_rnotify
 *
 * Description:
 *   Called each time that a Router Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Router
 *   Advertisement.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rnotify(FAR struct net_driver_s *dev);
#else
#  define icmpv6_rnotify(d) (0)
#endif

/****************************************************************************
 * Name: icmpv6_sock_initialize
 *
 * Description:
 *   Initialize the IPPROTO_ICMP socket connection structures.  Called once
 *   and only from the network initialization layer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
void icmpv6_sock_initialize(void);
#endif

/****************************************************************************
 * Name: icmpv6_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
FAR struct icmpv6_conn_s *icmpv6_alloc(void);
#endif

/****************************************************************************
 * Name: icmpv6_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
void icmpv6_free(FAR struct icmpv6_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmpv6_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
FAR struct icmpv6_conn_s *icmpv6_active(uint16_t id);
#endif

/****************************************************************************
 * Name: icmpv6_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
FAR struct icmpv6_conn_s *icmpv6_nextconn(FAR struct icmpv6_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmpv6_findconn
 *
 * Description:
 *   Find an ICMPv6 connection structure that is expecting a ICMPv6 ECHO
 *   response with this ID from this device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
FAR struct icmpv6_conn_s *icmpv6_findconn(FAR struct net_driver_s *dev,
                                          uint16_t id);
#endif

/****************************************************************************
 * Name: icmpv6_sendmsg
 *
 * Description:
 *   Implements the sendmsg() operation for the case of the IPPROTO_ICMP6
 *   socket.  The 'buf' parameter points to a block of memory that includes
 *   an ICMPv6 request header, followed by any payload that accompanies the
 *   request.  The 'len' parameter includes both the size of the ICMPv6
 *   header and the following payload.
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

#ifdef CONFIG_NET_ICMPv6_SOCKET
ssize_t icmpv6_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags);
#endif

/****************************************************************************
 * Name: icmpv6_recvmsg
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   data gram socket with the IPPROTO_ICMP6 protocol.  icmpv6_recvmsg()
 *   receives ICMPv6 ECHO replies for the a socket.
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

#ifdef CONFIG_NET_ICMPv6_SOCKET
ssize_t icmpv6_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags);
#endif

/****************************************************************************
 * Name: icmpv6_pollsetup
 *
 * Description:
 *   Setup to monitor events on one ICMPv6 socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP6 socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
int icmpv6_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Name: icmpv6_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an ICMPv6 socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP6 socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
int icmpv6_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Name: icmpv6_linkipaddr
 *
 * Description:
 *  Generate the device link scope ipv6 address as below:
 *  128  112  96   80    64   48   32   16
 *  ---- ---- ---- ----  ---- ---- ---- ----
 * fe80 0000 0000 0000 0000 00ff fe00 xx00 1-byte short addr IEEE 48-bit MAC
 * fe80 0000 0000 0000 0000 00ff fe00 xxxx 2-byte short addr IEEE 48-bit MAC
 * fe80 0000 0000 0000 xxxx xxff fexx xxxx 6-byte normal addr IEEE 48-bit MAC
 * fe80 0000 0000 0000 xxxx xxxx xxxx xxxx 8-byte extended addr IEEE EUI-64
 *
 * Input Parameters:
 *   dev    - The device driver structure containing the link layer address
 *   ipaddr - Receive the device link scope ipv6 address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_linkipaddr(FAR struct net_driver_s *dev, net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name: icmpv6_reply
 *
 * Description:
 *   Send an ICMPv6 message in response to a situation
 *   RFC 1122: 3.2.2 MUST send at least the IP header and 8 bytes of header.
 *       MAY send more (we do).
 *       MUST NOT change this header information.
 *       MUST NOT reply to a multicast/broadcast IP address.
 *       MUST NOT reply to a multicast/broadcast MAC address.
 *       MUST reply to only the first fragment.
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received packet
 *   type  - ICMPv6 Message Type, eg. ICMPv6_DEST_UNREACHABLE
 *   code  - ICMPv6 Message Code, eg. ICMPv6_PORT_UNREACH
 *   data  - Additional 32-bit parameter in the ICMPv6 header
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_reply(FAR struct net_driver_s *dev,
                  int type, int code, int data);

/****************************************************************************
 * Name: icmpv6_ioctl
 *
 * Description:
 *   This function performs icmp specific ioctl() operations.
 *
 * Parameters:
 *   conn     The ICMP connection of interest
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *   arglen   The length of 'arg'
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
int icmpv6_ioctl(FAR struct socket *psock,
                 int cmd, FAR void *arg, size_t arglen);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ICMPv6 && !CONFIG_NET_ICMPv6_NO_STACK */
#endif /* __NET_ICMPv6_ICMPv6_H */
