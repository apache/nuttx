/****************************************************************************
 * net/inet/inet.h
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET_IPv4
/* Increasing number used for the IP ID field. */

EXTERN uint16_t g_ipid;
#endif /* CONFIG_NET_IPv4 */

/* Well-known IPv6 addresses */

#ifdef CONFIG_NET_IPv6
EXTERN const net_ipv6addr_t g_ipv6_allzeroaddr; /* An address of all zeroes */
EXTERN const net_ipv6addr_t g_ipv6_allnodes;    /* All link local nodes */
#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER)
EXTERN const net_ipv6addr_t g_ipv6_allrouters;  /* All link local routers */
#ifdef CONFIG_NET_ICMPv6_AUTOCONF
EXTERN const net_ipv6addr_t g_ipv6_llnetmask;   /* Netmask for local link address */
#endif
#endif
#endif /* CONFIG_NET_IPv6 */

/* PF_INET/PF_INET6 socket address family interface */

#ifdef HAVE_INET_SOCKETS
EXTERN const struct sock_intf_s g_inet_sockif;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
struct tcp_conn_s; /* Forward reference */
#endif
struct socket; /* Forward reference */

/****************************************************************************
 * Name: inet_setipid
 *
 * Description:
 *   This function may be used at boot time to set the initial ip_id.
 *
 * Assumptions:
 *
 ****************************************************************************/

void inet_setipid(uint16_t id);

/****************************************************************************
 * Name: net_startmonitor
 *
 * Description:
 *   Set up to receive TCP connection state changes for a given socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *
 * Returned Value:
 *   On success, net_startmonitor returns OK; On any failure,
 *   net_startmonitor will return a negated errno value.  The only failure
 *   that can occur is if the socket has already been closed and, in this
 *   case, -ENOTCONN is returned.
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
int net_startmonitor(FAR struct socket *psock);
#endif

/****************************************************************************
 * Name: net_stopmonitor
 *
 * Description:
 *   Stop monitoring TCP connection changes for a given socket
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
void net_stopmonitor(FAR struct tcp_conn_s *conn);
#endif

/****************************************************************************
 * Name: net_lostconnection
 *
 * Description:
 *   Called when a loss-of-connection event has occurred.
 *
 * Parameters:
 *   psock    The TCP socket structure associated.
 *   flags    Set of connection events events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
void net_lostconnection(FAR struct socket *psock, uint16_t flags);
#endif

/****************************************************************************
 * Name: ipv4_getsockname and ipv6_sockname
 *
 * Description:
 *   The ipv4_getsockname() and ipv6_getsocknam() function retrieve the
 *   locally-bound name of the specified INET socket.
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
 * Name: inet_connect
 *
 * Description:
 *   inet_connect() connects the local socket referred to by the structure
 *   'psock' to the address specified by 'addr'. The addrlen argument
 *   specifies the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'psock'.
 *
 *   If the socket 'psock' is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully
 *   inet_connect() only once; connectionless protocol sockets may use
 *   inet_connect() multiple times to change their association.
 *   Connectionless sockets may dissolve the association by connecting to
 *   an address with the sa_family member of sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failue.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

int inet_connect(FAR struct socket *psock, FAR const struct sockaddr *addr,
                 socklen_t addrlen);

/****************************************************************************
 * Name: inet_sendfile
 *
 * Description:
 *   The inet_sendfile() call may be used only when the INET socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See sendfile() for a list
 *   appropriate error return values.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_SENDFILE) && defined(CONFIG_NET_TCP) && \
    !defined(CONFIG_NET_TCP_NO_STACK)
ssize_t inet_sendfile(FAR struct socket *psock, FAR struct file *infile,
                      FAR off_t *offset, size_t count);
#endif

/****************************************************************************
 * Name: inet_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  inet_recvfrom() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Parameters:
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

ssize_t inet_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen);

/****************************************************************************
 * Name: inet_close
 *
 * Description:
 *   Performs the close operation on an AF_INET or AF_INET6 socket instance
 *
 * Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int inet_close(FAR struct socket *psock);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __NET_INET_INET_H */
