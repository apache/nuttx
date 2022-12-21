/****************************************************************************
 * net/inet/inet.h
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

#ifndef __NET_INET_INET_H
#define __NET_INET_INET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#undef HAVE_INET_SOCKETS
#undef HAVE_PFINET_SOCKETS
#undef HAVE_PFINET6_SOCKETS

#if defined(CONFIG_NET_IPv4) || defined(CONFIG_NET_IPv6)
#  define HAVE_INET_SOCKETS

#  if defined(CONFIG_NET_IPv4)
#    define HAVE_PFINET_SOCKETS
#  endif

#  if defined(CONFIG_NET_IPv6)
#    define HAVE_PFINET6_SOCKETS
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Well-known IPv6 addresses */

#ifdef CONFIG_NET_IPv6
EXTERN const net_ipv6addr_t g_ipv6_unspecaddr;       /* An address of all zeroes */
EXTERN const net_ipv6addr_t g_ipv6_allnodes;         /* All link local nodes */

#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER) || \
    defined(CONFIG_NET_MLD)
/* IPv6 Multi-cast IP addresses.  See RFC 2375 */

EXTERN const net_ipv6addr_t g_ipv6_allrouters;       /* All link local routers */
#ifdef CONFIG_NET_ICMPv6_AUTOCONF
EXTERN const net_ipv6addr_t g_ipv6_llnetmask;        /* Netmask for local link address */
#endif
#ifdef CONFIG_NET_MLD
EXTERN const net_ipv6addr_t g_ipv6_allmldv2routers;  /* All MLDv2 link local routers */
#endif
#ifdef CONFIG_NET_ETHERNET
/* IPv6 Multi-cast Ethernet addresses.  Formed from the 16-bit prefix:
 *
 *   0x33:0x33:xx:xx:xx:xx:
 *
 * and the last 32-bits of the IPv6 IP address
 */

EXTERN const struct ether_addr g_ipv6_ethallnodes;    /* All link local nodes */
EXTERN const struct ether_addr g_ipv6_ethallrouters;  /* All link local routers */

#endif /* CONFIG_NET_ETHERNET */
#endif /* CONFIG_NET_ICMPv6_AUTOCONF || CONFIG_NET_ICMPv6_ROUTER || CONFIG_NET_MLD */
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
struct tcp_conn_s; /* Forward reference */
#endif
struct socket; /* Forward reference */

/****************************************************************************
 * Name: inet_sockif
 *
 * Description:
 *   Return the socket interface associated with the inet address family.
 *
 * Input Parameters:
 *   family   - Socket address family
 *   type     - Socket type
 *   protocol - Socket protocol
 *
 * Returned Value:
 *   On success, a non-NULL instance of struct sock_intf_s is returned.  NULL
 *   is returned only if the address family is not supported.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
inet_sockif(sa_family_t family, int type, int protocol);

/****************************************************************************
 * Name: ipv4_setsockopt and ipv6_setsockopt
 *
 * Description:
 *   ipv4/6_setsockopt() sets the IPv4- or IPv6-protocol socket option
 *   specified by the 'option' argument to the value pointed to by the
 *   'value' argument for the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of IPv4 and IPv6
 *   protocol socket options.
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

#ifdef CONFIG_NET_IPv4
int ipv4_setsockopt(FAR struct socket *psock, int option,
                    FAR const void *value, socklen_t value_len);
#endif
#ifdef CONFIG_NET_IPv6
int ipv6_setsockopt(FAR struct socket *psock, int option,
                    FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Name: ipv4_getsockopt
 *
 * Description:
 *   ipv4_getsockopt() retrieve the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   See <netinet/in.h> for the a complete list of values of IPv4 protocol
 *   socket options.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_getsockopt(FAR struct socket *psock, int option,
                    FAR void *value, FAR socklen_t *value_len);
#endif
#ifdef CONFIG_NET_IPv6
int ipv6_getsockopt(FAR struct socket *psock, int option,
                    FAR void *value, FAR socklen_t *value_len);
#endif

/****************************************************************************
 * Name: ipv4_getsockname and ipv6_sockname
 *
 * Description:
 *   The ipv4_getsockname() and ipv6_getsockname() function retrieve the
 *   locally-bound name of the specified INET socket.
 *
 * Input Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of returned error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_getsockname(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen);
#endif
#ifdef CONFIG_NET_IPv6
int ipv6_getsockname(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen);
#endif

/****************************************************************************
 * Name: ipv4_getpeername and ipv6_peername
 *
 * Description:
 *   The ipv4_getpeername() and ipv6_peername() function retrieve the
 *   remote-connected name of the specified INET socket.
 *
 * Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getpeername() for the list of returned error values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_getpeername(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen);
#endif
#ifdef CONFIG_NET_IPv6
int ipv6_getpeername(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen);
#endif

/****************************************************************************
 * Name: inet_close
 *
 * Description:
 *   Performs the close operation on an AF_INET or AF_INET6 socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int inet_close(FAR struct socket *psock);

/****************************************************************************
 * Name: inet_txdrain
 *
 * Description:
 *   Wait for any buffered Tx data to be sent from the socket.
 *
 * Parameters:
 *   psock   - Pointer to the socket structure instance
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated error value is returned on
 *   any failure.
 *
 ****************************************************************************/

int inet_txdrain(FAR struct socket *psock, unsigned int timeout);

/****************************************************************************
 * Name: ipv4_build_header
 *
 * Description:
 *   build IPv4 header
 *
 * Input Parameters:
 *   ipv4       Pointer to IPv4 header's buffer
 *   total_len  total length of the IPv4 packet
 *   prot       the next level protocol used in IPv4 packet
 *   src_ip     Source IPv4 address
 *   dst_ip     Destination IPv4 address
 *   ttl        Time to live(IPv4)
 *   tos        Type of Service(IPv4)
 *   opt        IPv4 options
 *
 * Returned Value:
 *   length of IPv4 header
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
uint16_t ipv4_build_header(FAR struct ipv4_hdr_s *ipv4, uint16_t total_len,
                           uint16_t prot, FAR const in_addr_t *src_ip,
                           FAR const in_addr_t *dst_ip, uint8_t ttl,
                           uint8_t tos, FAR struct ipv4_opt_s *opt);
#endif

/****************************************************************************
 * Name: ipv6_build_header
 *
 * Description:
 *   build IPv6 header
 *
 * Input Parameters:
 *   ipv6         Pointer to IPv6 header's buffer
 *   payload_len  Length of the IPv6 payload(without IPv6 header length)
 *   prot         Type of header immediately following the IPv6 header
 *   src_ip       Source IPv6 address
 *   dst_ip       Destination IPv6 address
 *   ttl          hop limit(IPv6)
 *   tclass       traffic class(IPv6)
 *
 * Returned Value:
 *   length of IPv6 header
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
uint16_t ipv6_build_header(FAR struct ipv6_hdr_s *ipv6, uint16_t payload_len,
                           uint16_t prot, const net_ipv6addr_t src_ip,
                           const net_ipv6addr_t dst_ip, uint8_t ttl,
                           uint8_t tclass);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __NET_INET_INET_H */
