/****************************************************************************
 * net/icmpv6/icmpv6.h
 *
 *   Copyright (C) 2015, 2017-2018 Gregory Nutt. All rights reserved.
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

#ifndef __NET_ICMPv6_ICMPv6_H
#define __NET_ICMPv6_ICMPv6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <semaphore.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new ICMPv6 data callback */

#define icmpv6_callback_alloc(dev) \
  devif_callback_alloc((dev), &(dev)->d_conncb)
#define icmpv6_callback_free(dev,cb) \
  devif_dev_callback_free((dev), (cb))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
/* Representation of a IPPROTO_ICMP socket connection */

struct devif_callback_s; /* Forward reference */

struct icmpv6_conn_s
{
  dq_entry_t node;     /* Supports a double linked list */
  uint16_t   id;       /* ICMPv6 ECHO request ID */
  uint8_t    nreqs;    /* Number of requests with no response received */
  uint8_t    crefs;    /* Reference counts on this instance */

  /* The device that the ICMPv6 request was sent on */

  FAR struct net_driver_s *dev;  /* Needed to free the callback structure */

#ifdef CONFIG_MM_IOB
  /* ICMPv6 response read-ahead list.  A singly linked list of type struct
   * iob_qentry_s where the ICMPv6 read-ahead data for the current ID is
   * retained.
   */

  struct iob_queue_s readahead;  /* Read-ahead buffering */
#endif
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

struct timespec;     /* Forward reference */
struct net_driver_s; /* Forward reference */
struct socket;       /* Forward reference */
struct sockaddr;     /* Forward reference */
struct pollfd;       /* Forward reference */

/****************************************************************************
 * Name: icmpv6_input
 *
 * Description:
 *   Handle incoming ICMPv6 input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received ICMPv6
 *         packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev);

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
 *   found in the Neighbor Table.  On error a negated errno value is returned:
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
void icmpv6_poll(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_solicit
 *
 * Description:
 *   Set up to send an ICMPv6 Neighbor Solicitation message
 *
 * Input Parameters:
 *   dev - Reference to an Ethernet device driver structure
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
 *   - The Ethernet header
 *   - The IPv6 header
 *   - The ICMPv6 Neighbor Router Message
 *
 *   The device IP address should have been set to the link local address
 *   prior to calling this function.
 *
 * Input Parameters:
 *   dev - Reference to an Ethernet device driver structure
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
 *   This function is called from icmpv6_neighbor() and executes in the normal
 *   tasking environment.
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
int icmpv6_wait(FAR struct icmpv6_notify_s *notify,
                FAR struct timespec *timeout);
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
 *   dev - The device driver structure to assign the address to
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
int icmpv6_rwait(FAR struct icmpv6_rnotify_s *notify,
                 FAR struct timespec *timeout);
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
 *   NOTE:  On success the network has the new address applied and is in
 *   the down state.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rnotify(FAR struct net_driver_s *dev, const net_ipv6addr_t draddr,
                    const net_ipv6addr_t prefix, unsigned int preflen);
#else
#  define icmpv6_rnotify(d,p,l)
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
 *   Find an ICMPv6 connection structure that is expecting a ICMPv6 ECHO response
 *   with this ID from this device
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
 * Name: icmpv6_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the IPPROTO_ICMP6
 *   socket.  The 'buf' parameter points to a block of memory that includes
 *   an ICMPv6 request header, followed by any payload that accompanies the
 *   request.  The 'len' parameter includes both the size of the ICMPv6
 *   header and the following payload.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send_to() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
ssize_t icmpv6_sendto(FAR struct socket *psock, FAR const void *buf, size_t len,
                      int flags, FAR const struct sockaddr *to, socklen_t tolen);
#endif

/****************************************************************************
 * Name: icmpv6_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET6
 *   data gram socket with the IPPROTO_ICMP6 protocol.  icmpv6_recvfrom()
 *   receives ICMPv6 ECHO replies for the a socket.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_SOCKET
ssize_t icmpv6_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                        int flags, FAR struct sockaddr *from,
                        FAR socklen_t *fromlen);
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ICMPv6 */
#endif /* __NET_ICMPv6_ICMPv6_H */
