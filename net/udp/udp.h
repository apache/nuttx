/****************************************************************************
 * net/udp/udp.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef __NET_UDP_UDP_H
#define __NET_UDP_UDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_UDP_READAHEAD
#  include <nuttx/net/iob.h>
#endif

#ifdef CONFIG_NET_UDP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Conditions for support UDP poll/select operations */

#if !defined(CONFIG_DISABLE_POLL) && CONFIG_NSOCKET_DESCRIPTORS > 0 && \
    defined(CONFIG_NET_UDP_READAHEAD)
#  define HAVE_UDP_POLL
#endif

/* Allocate a new UDP data callback */

#define udp_callback_alloc(conn)   devif_callback_alloc(&conn->list)
#define udp_callback_free(conn,cb) devif_callback_free(cb, &conn->list)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a uIP UDP connection */

struct devif_callback_s;  /* Forward reference */
struct udp_hdr_s;         /* Forward reference */

struct udp_conn_s
{
  dq_entry_t node;        /* Supports a doubly linked list */
  union ip_binding_u u;   /* IP address binding */
  uint16_t lport;         /* Bound local port number (network byte order) */
  uint16_t rport;         /* Remote port number (network byte order) */
  uint8_t  domain;        /* IP domain: PF_INET or PF_INET6 */
  uint8_t  ttl;           /* Default time-to-live */
  uint8_t  crefs;         /* Reference counts on this instance */

#ifdef CONFIG_NET_UDP_READAHEAD
  /* Read-ahead buffering.
   *
   *   readahead - A singly linked list of type struct iob_qentry_s
   *               where the UDP/IP read-ahead data is retained.
   */

  struct iob_queue_s readahead;   /* Read-ahead buffering */
#endif

  /* Defines the list of UDP callbacks */

  FAR struct devif_callback_s *list;
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct sockaddr;      /* Forward reference */
struct socket;        /* Forward reference */
struct net_driver_s;  /* Forward reference */
struct pollfd;        /* Forward reference */

/****************************************************************************
 * Name: udp_initialize
 *
 * Description:
 *   Initialize the UDP connection structures.  Called once and only from
 *   the UIP layer.
 *
 ****************************************************************************/

void udp_initialize(void);

/****************************************************************************
 * Name: udp_alloc
 *
 * Description:
 *   Allocate a new, uninitialized UDP connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_alloc(uint8_t domain);

/****************************************************************************
 * Name: udp_free
 *
 * Description:
 *   Free a UDP connection structure that is no longer in use. This should be
 *   done by the implementation of close().
 *
 ****************************************************************************/

void udp_free(FAR struct udp_conn_s *conn);

/****************************************************************************
 * Name: udp_active
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used within the provided UDP/IP header
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_active(FAR struct net_driver_s *dev,
                                  FAR struct udp_hdr_s *udp);

/****************************************************************************
 * Name: udp_nextconn
 *
 * Description:
 *   Traverse the list of allocated UDP connections
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

FAR struct udp_conn_s *udp_nextconn(FAR struct udp_conn_s *conn);

/****************************************************************************
 * Name: udp_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard UDP bind()
 *   operation.
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

int udp_bind(FAR struct udp_conn_s *conn, FAR const struct sockaddr *addr);

/****************************************************************************
 * Name: udp_connect
 *
 * Description:
 *   This function sets up a new UDP connection. The function will
 *   automatically allocate an unused local port for the new
 *   connection. However, another port can be chosen by using the
 *   udp_bind() call, after the udp_connect() function has been
 *   called.
 *
 *   This function is called as part of the implementation of sendto
 *   and recvfrom.
 *
 * Input Parameters:
 *   conn - A reference to UDP connection structure
 *   addr - The address of the remote host.
 *
 * Assumptions:
 *   This function is called user code.  Interrupts may be enabled.
 *
 ****************************************************************************/

int udp_connect(FAR struct udp_conn_s *conn, FAR const struct sockaddr *addr);

/****************************************************************************
 * Function: udp_ipv4_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv4 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void udp_ipv4_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: udp_ipv6_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv6 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void udp_ipv6_select(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: udp_poll
 *
 * Description:
 *   Poll a UDP "connection" structure for availability of TX data
 *
 * Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The UDP "connection" to poll for TX data
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void udp_poll(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn);

/****************************************************************************
 * Name: udp_send
 *
 * Description:
 *   Set-up to send a UDP packet
 *
 * Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The UDP "connection" structure holding port information
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void udp_send(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn);

/****************************************************************************
 * Name: udp_ipv4_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv4 packet
 *
 * Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Return:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int udp_ipv4_input(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: udp_ipv6_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv6 packet
 *
 * Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Return:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int udp_ipv6_input(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: udp_callback
 *
 * Description:
 *   Inform the application holding the UDP socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

uint16_t udp_callback(FAR struct net_driver_s *dev,
                      FAR struct udp_conn_s *conn, uint16_t flags);

/****************************************************************************
 * Function: psock_udp_sendto
 *
 * Description:
 *   This function implements the UDP-specific logic of the standard
 *   sendto() socket operation.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 *   NOTE: All input parameters were verified by sendto() before this
 *   function was called.
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See the description in
 *   net/socket/sendto.c for the list of appropriate return value.
 *
 ****************************************************************************/

ssize_t psock_udp_sendto(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags, FAR const struct sockaddr *to,
                         socklen_t tolen);

/****************************************************************************
 * Function: udp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one UDP/IP socket
 *
 * Input Parameters:
 *   psock - The UDP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef HAVE_UDP_POLL
int udp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Function: udp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an UDP/IP socket
 *
 * Input Parameters:
 *   psock - The TCP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef HAVE_UDP_POLL
int udp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_UDP */
#endif /* __NET_UDP_UDP_H */
