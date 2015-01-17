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

#ifdef CONFIG_NET_UDP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

  /* Defines the list of UDP callbacks */

  struct devif_callback_s *list;
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

struct net_driver_s;  /* Forward reference */
struct udp_iphdr_s;   /* Forward reference */

/* Defined in udp_conn.c ****************************************************/
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

#ifdef CONFIG_NET_IPv6
int udp_bind(FAR struct udp_conn_s *conn, FAR const struct sockaddr_in6 *addr);
#else
int udp_bind(FAR struct udp_conn_s *conn, FAR const struct sockaddr_in *addr);
#endif

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

#ifdef CONFIG_NET_IPv6
int udp_connect(FAR struct udp_conn_s *conn,
                FAR const struct sockaddr_in6 *addr);
#else
int udp_connect(FAR struct udp_conn_s *conn,
                FAR const struct sockaddr_in *addr);
#endif

/* Defined in udp_ipselect.c ************************************************/
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

/* Defined in udp_poll.c ****************************************************/
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

/* Defined in udp_send.c ****************************************************/
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

/* Defined in udp_input.c ***************************************************/
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

/* Defined in udp_callback.c ************************************************/
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_UDP */
#endif /* __NET_UDP_UDP_H */
