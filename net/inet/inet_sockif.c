/****************************************************************************
 * net/inet/inet_sockif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>
#include <nuttx/kmalloc.h>

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "sixlowpan/sixlowpan.h"
#include "socket/socket.h"
#include "inet/inet.h"

#ifdef HAVE_INET_SOCKETS

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

union sockaddr_u
{
  struct sockaddr     addr;
  struct sockaddr_in  inaddr;
  struct sockaddr_in6 in6addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)

static int        inet_setup(FAR struct socket *psock);
static sockcaps_t inet_sockcaps(FAR struct socket *psock);
static void       inet_addref(FAR struct socket *psock);
static int        inet_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        inet_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        inet_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        inet_listen(FAR struct socket *psock, int backlog);
static int        inet_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        inet_accept(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                    FAR struct socket *newsock);
static int        inet_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static ssize_t    inet_send(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags);
static ssize_t    inet_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen);
static ssize_t    inet_sendmsg(FAR struct socket *psock,
                    FAR struct msghdr *msg, int flags);
static ssize_t    inet_recvmsg(FAR struct socket *psock,
                    FAR struct msghdr *msg, int flags);
static int        inet_ioctl(FAR struct socket *psock,
                    int cmd, unsigned long arg);
static int        inet_socketpair(FAR struct socket *psocks[2]);
static int        inet_shutdown(FAR struct socket *psock, int how);
#ifdef CONFIG_NET_SOCKOPTS
static int        inet_getsockopt(FAR struct socket *psock, int level,
                    int option, FAR void *value, FAR socklen_t *value_len);
static int        inet_setsockopt(FAR struct socket *psock, int level,
                    int option, FAR const void *value, socklen_t value_len);
#endif
#ifdef CONFIG_NET_SENDFILE
static ssize_t    inet_sendfile(FAR struct socket *psock,
                    FAR struct file *infile, FAR off_t *offset,
                    size_t count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sock_intf_s g_inet_sockif =
{
  inet_setup,       /* si_setup */
  inet_sockcaps,    /* si_sockcaps */
  inet_addref,      /* si_addref */
  inet_bind,        /* si_bind */
  inet_getsockname, /* si_getsockname */
  inet_getpeername, /* si_getpeername */
  inet_listen,      /* si_listen */
  inet_connect,     /* si_connect */
  inet_accept,      /* si_accept */
  inet_poll,        /* si_poll */
  inet_sendmsg,     /* si_sendmsg */
  inet_recvmsg,     /* si_recvmsg */
  inet_close,       /* si_close */
  inet_ioctl,       /* si_ioctl */
  inet_socketpair,  /* si_socketpair */
  inet_shutdown     /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , inet_getsockopt /* si_getsockopt */
  , inet_setsockopt /* si_setsockopt */
#endif
#ifdef CONFIG_NET_SENDFILE
  , inet_sendfile   /* si_sendfile */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_tcp_alloc
 *
 * Description:
 *   Allocate and attach a TCP connection structure.
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static int inet_tcp_alloc(FAR struct socket *psock)
{
  /* Allocate the TCP connection structure */

  FAR struct tcp_conn_s *conn = tcp_alloc(psock->s_domain);
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      nerr("ERROR: Failed to reserve TCP connection structure\n");
      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* It is expected the socket has not yet been associated with
   * any other connection.
   */

  DEBUGASSERT(psock->s_conn == NULL);

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* NET_TCP_HAVE_STACK */

/****************************************************************************
 * Name: inet_udp_alloc
 *
 * Description:
 *   Allocate and attach a UDP connection structure.
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static int inet_udp_alloc(FAR struct socket *psock)
{
  /* Allocate the UDP connection structure */

  FAR struct udp_conn_s *conn = udp_alloc(psock->s_domain);
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      nerr("ERROR: Failed to reserve UDP connection structure\n");
      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 *   NOTE:  This is common logic for both the AF_INET and AF_INET6 address
 *   families.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int inet_setup(FAR struct socket *psock)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   *
   * REVISIT:  Only SOCK_STREAM and SOCK_DGRAM are supported.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        if (psock->s_proto != 0 && psock->s_proto != IPPROTO_TCP)
          {
            nerr("ERROR: Unsupported stream protocol: %d\n", psock->s_proto);
            return -EPROTONOSUPPORT;
          }

#ifdef NET_TCP_HAVE_STACK
        /* Allocate and attach the TCP connection structure */

        return inet_tcp_alloc(psock);
#else
        nwarn("WARNING:  SOCK_STREAM disabled\n");
        return -ENETDOWN;
#endif
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        if (psock->s_proto != 0 && psock->s_proto != IPPROTO_UDP)
          {
            nerr("ERROR: Unsupported datagram protocol: %d\n",
                 psock->s_proto);
            return -EPROTONOSUPPORT;
          }

#ifdef NET_UDP_HAVE_STACK
        /* Allocate and attach the UDP connection structure */

        return inet_udp_alloc(psock);
#else
        nwarn("WARNING:  SOCK_DGRAM disabled\n");
        return -ENETDOWN;
#endif
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
      case SOCK_CTRL:
#  ifdef NET_TCP_HAVE_STACK
        if (psock->s_proto == 0 || psock->s_proto == IPPROTO_TCP)
          {
             /* Allocate and attach the TCP connection structure */

             return inet_tcp_alloc(psock);
          }

#  endif
#  ifdef NET_UDP_HAVE_STACK
        if (psock->s_proto == 0 || psock->s_proto == IPPROTO_UDP)
          {
             /* Allocate and attach the UDP connection structure */

             return inet_udp_alloc(psock);
          }

#  endif
        nerr("ERROR: Unsupported control protocol: %d\n", psock->s_proto);
        return -EPROTONOSUPPORT;
#endif /* CONFIG_NET_TCP || CONFIG_NET_UDP */

      default:
        nerr("ERROR: Unsupported type: %d\n", psock->s_type);
        return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: inet_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t inet_sockcaps(FAR struct socket *psock)
{
  switch (psock->s_type)
    {
#ifdef NET_TCP_HAVE_STACK
      case SOCK_STREAM:
        return SOCKCAP_NONBLOCKING;
#endif

#ifdef NET_UDP_HAVE_STACK
      case SOCK_DGRAM:
        return SOCKCAP_NONBLOCKING;
#endif

#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
      case SOCK_CTRL:
        return SOCKCAP_NONBLOCKING;
#endif

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: inet_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void inet_addref(FAR struct socket *psock)
{
  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

#ifdef NET_TCP_HAVE_STACK
  if (psock->s_type == SOCK_STREAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_TCP)))
    {
      FAR struct tcp_conn_s *conn = psock->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
#ifdef NET_UDP_HAVE_STACK
  if (psock->s_type == SOCK_DGRAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_UDP)))
    {
      FAR struct udp_conn_s *conn = psock->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
    {
      nerr("ERROR: Unsupported type: %d\n", psock->s_type);
    }
}

/****************************************************************************
 * Name: inet_bind
 *
 * Description:
 *   inet_bind() gives the socket 'psock' the local address 'addr'.  'addr'
 *   is 'addrlen' bytes long.  Traditionally, this is called "assigning a
 *   name to a socket."  When a socket is created with socket(), it exists
 *   in a name space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int inet_bind(FAR struct socket *psock,
                     FAR const struct sockaddr *addr, socklen_t addrlen)
{
  int minlen;
  int ret;

  /* Verify that a valid address has been provided */

  switch (addr->sa_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      minlen = sizeof(struct sockaddr_in);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      minlen = sizeof(struct sockaddr_in6);
      break;
#endif

    default:
      nerr("ERROR: Unrecognized address family: %d\n", addr->sa_family);
      return -EAFNOSUPPORT;
    }

  if (addrlen < minlen)
    {
      nerr("ERROR: Invalid address length: %d < %d\n", addrlen, minlen);
      return -EINVAL;
    }

  /* Perform the binding depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
#ifdef NET_TCP_HAVE_STACK
          /* Bind a TCP/IP stream socket. */

          ret = tcp_bind(psock->s_conn, addr);
#else
          nwarn("WARNING: TCP/IP stack is not available in this "
                "configuration\n");

          ret = -ENOSYS;
#endif
        }
        break;
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
#ifdef NET_UDP_HAVE_STACK
          /* Bind a UDP/IP datagram socket */

          ret = udp_bind(psock->s_conn, addr);
#else
          nwarn("WARNING: UDP stack is not available in this "
                "configuration\n");
          ret = -ENOSYS;
#endif
        }
        break;
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
      case SOCK_CTRL:
        {
          nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
          ret = -EOPNOTSUPP;
        }
        break;
#endif

      default:
        nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
        ret = -EBADF;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: inet_getsockname
 *
 * Description:
 *   The inet_getsockname() function retrieves the locally-bound name of
 *   the specified INET socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int inet_getsockname(FAR struct socket *psock,
                            FAR struct sockaddr *addr,
                            FAR socklen_t *addrlen)
{
  /* Handle by address domain */

  switch (psock->s_domain)
    {
#ifdef CONFIG_NET_IPv4
    case PF_INET:
      return ipv4_getsockname(psock, addr, addrlen);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case PF_INET6:
      return ipv6_getsockname(psock, addr, addrlen);
      break;
#endif

    default:
      return -EAFNOSUPPORT;
    }
}

/****************************************************************************
 * Name: inet_getpeername
 *
 * Description:
 *   The inet_getpeername() function retrieves the remote-connected name of
 *   the specified INET socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getpeername() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int inet_getpeername(FAR struct socket *psock,
                            FAR struct sockaddr *addr,
                            FAR socklen_t *addrlen)
{
  /* Handle by address domain */

  switch (psock->s_domain)
    {
#ifdef CONFIG_NET_IPv4
    case PF_INET:
      return ipv4_getpeername(psock, addr, addrlen);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case PF_INET6:
      return ipv6_getpeername(psock, addr, addrlen);
      break;
#endif

    default:
      return -EAFNOSUPPORT;
    }
}

#ifdef CONFIG_NET_SOCKOPTS

/****************************************************************************
 * Name: inet_get_socketlevel_option
 *
 * Description:
 *   inet_get_socketlevel_option() retrieve the value for the option
 *   specified by the 'option' argument for the socket specified by the
 *   'psock' argument.  If the size of the option value is greater than
 *   'value_len', the value stored in the object pointed to by the 'value'
 *   argument will be silently truncated. Otherwise, the length pointed to
 *   by the 'value_len' argument will be modified to indicate the actual
 *   length of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET; to retrieve options at the TCP-protocol level, the level
 *   argument is SOL_TCP.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netinet/tcp.h for the case of the TCP protocol).
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

static int inet_get_socketlevel_option(FAR struct socket *psock, int option,
                                       FAR void *value,
                                       FAR socklen_t *value_len)
{
  switch (option)
    {
#if CONFIG_NET_RECV_BUFSIZE > 0
      case SO_RCVBUF:     /* Reports receive buffer size */
        {
          if (*value_len != sizeof(int))
            {
              return -EINVAL;
            }

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp = psock->s_conn;
              *(FAR int *)value = tcp->rcv_bufs;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp = psock->s_conn;
              *(FAR int *)value = udp->rcvbufs;
            }
          else
#endif
            {
              return -ENOPROTOOPT;
            }
        }
        break;
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:     /* Reports send buffer size */
        {
          if (*value_len != sizeof(int))
            {
              return -EINVAL;
            }

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp = psock->s_conn;
              *(FAR int *)value = tcp->snd_bufs;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp = psock->s_conn;

              /* Save the send buffer size */

              *(FAR int *)value = udp->sndbufs;
            }
          else
#endif
            {
              return -ENOPROTOOPT;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      case SO_KEEPALIVE:
        {
          /* Any connection-oriented protocol could potentially support
           * SO_KEEPALIVE.  However, this option is currently only available
           * for TCP/IP.
           *
           * NOTE: SO_KEEPALIVE is not really a socket-level option; it is a
           * protocol-level option.  A given TCP connection may service
           * multiple sockets (via dup'ing of the socket). There is, however,
           * still only one connection to be monitored and that is a global
           * attribute across all of the clones that may use the underlying
           * connection.
           */

          /* Verifies TCP connections active by enabling the periodic
           * transmission of probes.
           */

          return tcp_getsockopt(psock, option, value, value_len);
        }
#endif

      default:
        return -ENOPROTOOPT;
    }

  return OK;
}

/****************************************************************************
 * Name: inet_getsockopt
 *
 * Description:
 *   inet_getsockopt() retrieve the value for the option specified by the
 *   'option' argument at the protocol level specified by the 'level'
 *   argument. If the size of the option value is greater than 'value_len',
 *   the value stored in the object pointed to by the 'value' argument will
 *   be silently truncated. Otherwise, the length pointed to by the
 *   'value_len' argument will be modified to indicate the actual length
 *   of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

static int inet_getsockopt(FAR struct socket *psock, int level, int option,
                           FAR void *value, FAR socklen_t *value_len)
{
  switch (level)
    {
      case SOL_SOCKET:
        return inet_get_socketlevel_option(psock, option, value, value_len);

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      case IPPROTO_TCP:
        return tcp_getsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv4
      case IPPROTO_IP:/* IPv4 protocol socket options (see include/netinet/in.h) */
        return ipv4_getsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv6
      case IPPROTO_IPV6:/* IPv6 protocol socket options (see include/netinet/in.h) */
          return ipv6_getsockopt(psock, option, value, value_len);
#endif

      default:
        return -ENOPROTOOPT;
    }
}

/****************************************************************************
 * Name: inet_set_socketlevel_option
 *
 * Description:
 *   inet_set_socketlevel_option() sets the socket-level option specified by
 *   the 'option' argument to the value pointed to by the 'value' argument
 *   for the socket specified by the 'psock' argument.
 *
 *   See <sys/socket.h> a complete list of values for the socket level
 *   'option' argument.
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

static int inet_set_socketlevel_option(FAR struct socket *psock, int option,
                                       FAR const void *value,
                                       socklen_t value_len)
{
  switch (option)
    {
#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      case SO_KEEPALIVE:
        {
          /* Any connection-oriented protocol could potentially support
           * SO_KEEPALIVE.  However, this option is currently only available
           * for TCP/IP.
           *
           * NOTE: SO_KEEPALIVE is not really a socket-level option; it is a
           * protocol-level option.  A given TCP connection may service
           * multiple sockets (via dup'ing of the socket). There is, however,
           * still only one connection to be monitored and that is a global
           * attribute across all of the clones that may use the underlying
           * connection.
           */

          /* Verifies TCP connections active by enabling the
           * periodic transmission of probes
           */

          return tcp_setsockopt(psock, option, value, value_len);
        }
#endif

#ifdef CONFIG_NET_SOLINGER
      case SO_LINGER:
        {
          /* Lingers on a close() if data is present */

          FAR struct socket_conn_s *conn = psock->s_conn;
          FAR struct linger *setting;

          /* Verify that option is at least the size of an 'struct linger'. */

          if (value_len < sizeof(struct linger))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          setting = (FAR struct linger *)value;

          /* Lock the network so that we have exclusive access to the socket
           * options.
           */

          net_lock();

          /* Set or clear the linger option bit and linger time
           * (in deciseconds)
           */

          if (setting->l_onoff)
            {
              _SO_SETOPT(conn->s_options, option);
              conn->s_linger = 10 * setting->l_linger;
            }
          else
            {
              _SO_CLROPT(conn->s_options, option);
              conn->s_linger = 0;
            }

          net_unlock();
        }
        break;
#endif

#if CONFIG_NET_RECV_BUFSIZE > 0
      case SO_RCVBUF:     /* Sets receive buffer size */
        {
          int buffersize;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          buffersize = *(FAR int *)value;
          if (buffersize < 0)
            {
              return -EINVAL;
            }

          net_lock();

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp = psock->s_conn;

              /* Save the receive buffer size */

              tcp->rcv_bufs = buffersize;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp = psock->s_conn;

              /* Save the receive buffer size */

              udp->rcvbufs = buffersize;
            }
          else
#endif
            {
              net_unlock();
              return -ENOPROTOOPT;
            }

          net_unlock();
        }
        break;
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:     /* Sets send buffer size */
        {
          int buffersize;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          buffersize = *(FAR int *)value;

          if (buffersize < 0)
            {
              return -EINVAL;
            }

          net_lock();

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp = psock->s_conn;

              /* Save the send buffer size */

              tcp->snd_bufs = buffersize;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp = psock->s_conn;

              /* Save the send buffer size */

              udp->sndbufs = buffersize;
            }
          else
#endif
            {
              net_unlock();
              return -ENOPROTOOPT;
            }

          net_unlock();
        }
        break;
#endif

      default:
        return -ENOPROTOOPT;
    }

  return OK;
}

/****************************************************************************
 * Name: inet_setsockopt
 *
 * Description:
 *   inet_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the connection.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

static int inet_setsockopt(FAR struct socket *psock, int level, int option,
                           FAR const void *value, socklen_t value_len)
{
  switch (level)
    {
      case SOL_SOCKET:
        return inet_set_socketlevel_option(psock, option, value, value_len);

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      case IPPROTO_TCP:/* TCP protocol socket options (see include/netinet/tcp.h) */
        return tcp_setsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_UDPPROTO_OPTIONS
      case IPPROTO_UDP:/* UDP protocol socket options (see include/netinet/udp.h) */
        return udp_setsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv4
      case IPPROTO_IP:/* IPv4 protocol socket options (see include/netinet/in.h) */
        return ipv4_setsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv6
      case IPPROTO_IPV6:/* IPv6 protocol socket options (see include/netinet/in.h) */
        return ipv6_setsockopt(psock, option, value, value_len);
#endif
      default:
        return -ENOPROTOOPT;
    }
}

#endif

/****************************************************************************
 * Name: inet_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of AFINET
 *   and AFINET6 sockets, psock_listen() calls this function.  The
 *   psock_listen() call applies only to sockets of type SOCK_STREAM or
 *   SOCK_SEQPACKET.
 *
 * Input Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See listen() for the set of appropriate error values.
 *
 ****************************************************************************/

int inet_listen(FAR struct socket *psock, int backlog)
{
#if defined(CONFIG_NET_TCP) && defined(NET_TCP_HAVE_STACK)
  FAR struct tcp_conn_s *conn;
  int ret;
#endif

  /* Verify that the sockfd corresponds to a connected SOCK_STREAM */

  if (psock->s_type != SOCK_STREAM)
    {
      nerr("ERROR:  Unsupported socket type: %d\n",
           psock->s_type);
      return -EOPNOTSUPP;
    }

#ifdef CONFIG_NET_TCP
#ifdef NET_TCP_HAVE_STACK
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  if (conn->lport <= 0)
    {
      return -EOPNOTSUPP;
    }

#ifdef CONFIG_NET_TCPBACKLOG
  /* Set up the backlog for this connection */

  ret = tcp_backlogcreate(conn, backlog);
  if (ret < 0)
    {
      nerr("ERROR: tcp_backlogcreate failed: %d\n", ret);
      return ret;
    }
#endif

  /* Start listening to the bound port.  This enables callbacks when
   * accept() is called and enables poll()/select() logic.
   */

  ret = tcp_listen(conn);
  if (ret < 0)
    {
      nerr("ERROR: tcp_listen failed: %d\n", ret);
    }

  return ret;
#else
  nwarn("WARNING:  Stream socket support not available\n");
  return -EOPNOTSUPP;
#endif /* NET_TCP_HAVE_STACK */
#else
  nwarn("WARNING:  Stream socket support not enabled\n");
  return -EOPNOTSUPP;
#endif /* CONFIG_NET_TCP */
}

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
 * Input Parameters:
 *   psock   - Pointer to a socket structure initialized by psock_socket()
 *   addr    - Server address (form depends on type of socket).  The upper
 *             socket layer has verified that this address is non-NULL.
 *   addrlen - Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int inet_connect(FAR struct socket *psock,
                        FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR const struct sockaddr_in *inaddr =
    (FAR const struct sockaddr_in *)addr;

  /* Verify that a valid address has been provided */

  switch (inaddr->sin_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      {
        if (addrlen < sizeof(struct sockaddr_in))
          {
            return -EINVAL;
          }
      }
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      {
        if (addrlen < sizeof(struct sockaddr_in6))
          {
            return -EINVAL;
          }
      }
      break;
#endif

    default:
      DEBUGPANIC();
      return -EAFNOSUPPORT;
    }

  /* Perform the connection depending on the protocol type */

  switch (psock->s_type)
    {
#if defined(CONFIG_NET_TCP) && defined(NET_TCP_HAVE_STACK)
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *conn = psock->s_conn;

          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(conn->sconn.s_flags))
            {
              return -EISCONN;
            }

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          if (conn->domain != addr->sa_family)
            {
              nerr("conn's domain must be the same as addr's family!\n");
              return -EPROTOTYPE;
            }

#endif
          /* It's not ... Connect the TCP/IP socket */

          return psock_tcp_connect(psock, addr);
        }
#endif /* CONFIG_NET_TCP */

#if defined(CONFIG_NET_UDP) && defined(NET_UDP_HAVE_STACK)
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *conn;
          int ret;

          /* We will accept connecting to a addr == NULL for disconnection.
           * However, the correct way is to disconnect is to provide an
           * address with sa_family == AF_UNSPEC.
           */

          if (addr != NULL && addr->sa_family == AF_UNSPEC)
            {
              addr = NULL;
            }

          /* Perform the connect/disconnect operation */

          conn = (FAR struct udp_conn_s *)psock->s_conn;
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          if (conn->domain != addr->sa_family)
            {
              nerr("conn's domain must be the same as addr's family!\n");
              return -EPROTOTYPE;
            }

#endif
          ret  = udp_connect(conn, addr);
          if (ret < 0 || addr == NULL)
            {
              /* Failed to connect or explicitly disconnected */

              conn->flags &= ~_UDP_FLAG_CONNECTMODE;
            }
          else
            {
              /* Successfully connected */

              conn->flags |= _UDP_FLAG_CONNECTMODE;
            }

          return ret;
        }
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP) && defined(CONFIG_NET_UDP)
      case SOCK_CTRL:
        {
          nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
          return -EOPNOTSUPP;
        }
#endif

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Name: inet_accept
 *
 * Description:
 *   The inet_accept function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an inet_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, inet_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, inet_accept returns
 *   EAGAIN.
 *
 * Input Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of
 *            'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a description of the appropriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int inet_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                       FAR socklen_t *addrlen, FAR struct socket *newsock)
{
#if defined(CONFIG_NET_TCP) && defined(NET_TCP_HAVE_STACK)
  int ret;
#endif

  /* Is the socket a stream? */

  if (psock->s_type != SOCK_STREAM)
    {
      nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
      return -EOPNOTSUPP;
    }

  /* Verify that a valid memory block has been provided to receive the
   * address
   */

  if (addr != NULL)
    {
      /* If an address is provided, then the length must also be provided. */

      DEBUGASSERT(*addrlen > 0);

      /* A valid length depends on the address domain */

      switch (psock->s_domain)
        {
#ifdef CONFIG_NET_IPv4
        case PF_INET:
          {
            if (*addrlen < sizeof(struct sockaddr_in))
              {
                return -EINVAL;
              }
          }
          break;
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
        case PF_INET6:
          {
            if (*addrlen < sizeof(struct sockaddr_in6))
              {
                return -EINVAL;
              }
          }
          break;
#endif /* CONFIG_NET_IPv6 */

        default:
          DEBUGPANIC();
          return -EINVAL;
        }
    }

  /* Initialize the socket structure. */

  newsock->s_domain = psock->s_domain;
  newsock->s_type   = SOCK_STREAM;
  newsock->s_sockif = psock->s_sockif;

  /* Perform the correct accept operation for this address domain */

#ifdef CONFIG_NET_TCP
#ifdef NET_TCP_HAVE_STACK
  /* Perform the local accept operation (the network locked must be locked
   * by the caller).
   */

  ret = psock_tcp_accept(psock, addr, addrlen, &newsock->s_conn);
  if (ret < 0)
    {
      nerr("ERROR: psock_tcp_accept failed: %d\n", ret);
      return ret;
    }

   /* Begin monitoring for TCP connection events on the newly connected
    * socket
    */

  ret = tcp_start_monitor(newsock);
  if (ret < 0)
    {
      /* tcp_start_monitor() can only fail on certain race conditions where
       * the connection was lost just before this function was called.  Undo
       * everything we have done and return a failure.
       */

      psock_close(newsock);
    }

  return ret;

#else
  nwarn("WARNING: SOCK_STREAM not supported in this configuration\n");
  return -EOPNOTSUPP;
#endif /* NET_TCP_HAVE_STACK */

#else
  nwarn("WARNING: TCP/IP not supported in this configuration\n");
  return -EOPNOTSUPP;
#endif /* CONFIG_NET_TCP */
}

/****************************************************************************
 * Name: inet_pollsetup
 *
 * Description:
 *   Setup to monitor events on one socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
static inline int inet_pollsetup(FAR struct socket *psock,
                                 FAR struct pollfd *fds)
{
#ifdef NET_TCP_HAVE_STACK
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollsetup(psock, fds);
    }
  else
#endif /* NET_TCP_HAVE_STACK */
#ifdef NET_UDP_HAVE_STACK
  if (psock->s_type != SOCK_STREAM)
    {
      return udp_pollsetup(psock, fds);
    }
  else
#endif /* NET_UDP_HAVE_STACK */
#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
  if (psock->s_type == SOCK_CTRL)
    {
      nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
      return -EOPNOTSUPP;
    }
  else
#endif
    {
      return -ENOSYS;
    }
}
#endif /* NET_TCP_HAVE_STACK || NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an socket
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

#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
static inline int inet_pollteardown(FAR struct socket *psock,
                                    FAR struct pollfd *fds)
{
#ifdef NET_TCP_HAVE_STACK
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollteardown(psock, fds);
    }
  else
#endif /* NET_TCP_HAVE_STACK */
#ifdef NET_UDP_HAVE_STACK
  if (psock->s_type == SOCK_DGRAM)
    {
      return udp_pollteardown(psock, fds);
    }
  else
#endif /* NET_UDP_HAVE_STACK */
#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
  if (psock->s_type == SOCK_CTRL)
    {
      nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
      return -EOPNOTSUPP;
    }
  else
#endif
    {
      return -ENOSYS;
    }
}
#endif /* NET_TCP_HAVE_STACK || NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: inet_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to net_poll which, indiectly, calls to function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int inet_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                     bool setup)
{
#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the TCP/IP poll() setup */

      return inet_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the TCP/IP poll() teardown */

      return inet_pollteardown(psock, fds);
    }
#else
    {
      return -ENOSYS;
    }
#endif /* NET_TCP_HAVE_STACK || !NET_UDP_HAVE_STACK */
}

/****************************************************************************
 * Name: inet_send
 *
 * Description:
 *   The inet_send() call may be used only when the socket is in a connected
 *   state  (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t inet_send(FAR struct socket *psock, FAR const void *buf,
                         size_t len, int flags)
{
#ifdef NET_UDP_HAVE_STACK
  FAR struct socket_conn_s *conn = psock->s_conn;
#endif
  ssize_t ret;

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
#ifdef CONFIG_NET_6LOWPAN
          /* Try 6LoWPAN TCP packet send */

          ret = psock_6lowpan_tcp_send(psock, buf, len);

#ifdef NET_TCP_HAVE_STACK
          if (ret < 0)
            {
              /* TCP/IP packet send */

              ret = psock_tcp_send(psock, buf, len, flags);
            }
#endif /* NET_TCP_HAVE_STACK */

#elif defined(NET_TCP_HAVE_STACK)
          ret = psock_tcp_send(psock, buf, len, flags);
#else
          ret = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
        }
        break;
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
#if defined(CONFIG_NET_6LOWPAN)
           /* Try 6LoWPAN UDP packet send */

           ret = psock_6lowpan_udp_send(psock, buf, len);

#ifdef NET_UDP_HAVE_STACK
          if (ret < 0)
            {
              /* UDP/IP packet send */

              ret = _SS_ISCONNECTED(conn->s_flags) ?
                psock_udp_sendto(psock, buf, len, 0, NULL, 0) : -ENOTCONN;
            }
#endif /* NET_UDP_HAVE_STACK */

#elif defined(NET_UDP_HAVE_STACK)
          /* Only UDP/IP packet send */

          ret = _SS_ISCONNECTED(conn->s_flags) ?
            psock_udp_sendto(psock, buf, len, 0, NULL, 0) : -ENOTCONN;
#else
          ret = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
        }
        break;
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
      case SOCK_CTRL:
        {
          nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
          ret = -EOPNOTSUPP;
        }
        break;
#endif

      default:
        {
          /* EDESTADDRREQ.  Signifies that the socket is not connection-mode
           * and no peer address is set.
           */

          nerr("ERROR:  Bad socket type: %d\n", psock->s_type);
          ret = -EDESTADDRREQ;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: inet_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the AF_INET and
 *   AF_INET6 sockets.
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

static ssize_t inet_sendto(FAR struct socket *psock, FAR const void *buf,
                           size_t len, int flags,
                           FAR const struct sockaddr *to, socklen_t tolen)
{
  socklen_t minlen;
  ssize_t nsent;

  /* Verify that a valid address has been provided */

  switch (to->sa_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      minlen = sizeof(struct sockaddr_in);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      minlen = sizeof(struct sockaddr_in6);
      break;
#endif

    default:
      nerr("ERROR: Unrecognized address family: %d\n", to->sa_family);
      return -EAFNOSUPPORT;
    }

  if (tolen < minlen)
    {
      nerr("ERROR: Invalid address length: %d < %d\n", tolen, minlen);
      return -EBADF;
    }

#ifdef CONFIG_NET_UDP
  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR: Connected socket\n");
      return -EBADF;
    }

  /* Now handle the INET sendto() operation */

#if defined(CONFIG_NET_6LOWPAN)
  /* Try 6LoWPAN UDP packet sendto() */

  nsent = psock_6lowpan_udp_sendto(psock, buf, len, flags, to, minlen);

#ifdef NET_UDP_HAVE_STACK
  if (nsent < 0)
    {
      /* UDP/IP packet sendto */

      nsent = psock_udp_sendto(psock, buf, len, flags, to, tolen);
    }
#endif /* NET_UDP_HAVE_STACK */

#elif defined(NET_UDP_HAVE_STACK)
  nsent = psock_udp_sendto(psock, buf, len, flags, to, tolen);
#else
  nwarn("WARNING: UDP not available in this configuiration\n");
  nsent = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
#else
  nwarn("WARNING: UDP not enabled in this configuiration\n");
  nsent = -EISCONN;
#endif /* CONFIG_NET_UDP */

  return nsent;
}

/****************************************************************************
 * Name: inet_sendmsg
 *
 * Description:
 *   The inet_send() call may be used only when the socket is in a connected
 *   state  (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      Message to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t inet_sendmsg(FAR struct socket *psock,
                            FAR struct msghdr *msg, int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR const struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  FAR const struct iovec *iov;
  FAR const struct iovec *end;
  int ret;

  if (msg->msg_iovlen == 1)
    {
      return to ? inet_sendto(psock, buf, len, flags, to, tolen) :
                  inet_send(psock, buf, len, flags);
    }

  end = &msg->msg_iov[msg->msg_iovlen];
  for (len = 0, iov = msg->msg_iov; iov != end; iov++)
    {
      len += iov->iov_len;
    }

  buf = kmm_malloc(len);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  for (len = 0, iov = msg->msg_iov; iov != end; iov++)
    {
      memcpy(((unsigned char *)buf) + len, iov->iov_base, iov->iov_len);
      len += iov->iov_len;
    }

  ret = to ? inet_sendto(psock, buf, len, flags, to, tolen) :
             inet_send(psock, buf, len, flags);

  kmm_free(buf);

  return ret;
}

/****************************************************************************
 * Name: inet_ioctl
 *
 * Description:
 *   This function performs network device specific operations.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

static int inet_ioctl(FAR struct socket *psock, int cmd, unsigned long arg)
{
  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
  if (psock->s_type == SOCK_STREAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_TCP)))
    {
      return tcp_ioctl(psock->s_conn, cmd, arg);
    }
#endif

#if defined(CONFIG_NET_UDP) && defined(NET_UDP_HAVE_STACK)
  if (psock->s_type == SOCK_DGRAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_UDP)))
    {
      return udp_ioctl(psock->s_conn, cmd, arg);
    }
#endif

  return -EINVAL;
}

/****************************************************************************
 * Name: inet_socketpair
 *
 * Description:
 *   Create a pair of connected sockets between psocks[2]
 *
 * Parameters:
 *   psocks   A reference to the socket structure of the socket pair
 *
 ****************************************************************************/

static int inet_socketpair(FAR struct socket *psocks[2])
{
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  FAR struct socket *pserver = psocks[1];
#if defined(CONFIG_NET_TCP)
  FAR struct socket server;
#endif
  union sockaddr_u addr[2];
  socklen_t len;
  int ret;

  /* Set the sock address to localhost */

#ifdef CONFIG_NET_IPv6
  if (psocks[0]->s_domain == AF_INET6)
    {
      struct in6_addr init_sin6_addr = IN6ADDR_LOOPBACK_INIT;

      len = sizeof(addr[0].in6addr);
      memset(&addr[0], 0, len);
      addr[0].in6addr.sin6_family = psocks[0]->s_domain;
      addr[0].in6addr.sin6_addr = init_sin6_addr;
    }
  else
#endif
    {
      len = sizeof(addr[0].inaddr);
      memset(&addr[0], 0, len);
      addr[0].inaddr.sin_family = psocks[0]->s_domain;
      addr[0].inaddr.sin_addr.s_addr = HTONL(INADDR_LOOPBACK);
    }

  memcpy(&addr[1], &addr[0], len);

  ret = psock_bind(psocks[0], &addr[0].addr, len);
  if (ret < 0)
    {
      return ret;
    }

  psock_getsockname(psocks[0], &addr[0].addr, &len);

  /* For SOCK_STREAM, Use proxy service handle to make temporary
   * pserver process, psocks[1] will be replaced with a new accept handle
   */

#if defined(CONFIG_NET_TCP)
  if (psocks[0]->s_type == SOCK_STREAM)
    {
      ret = psock_socket(psocks[1]->s_domain, psocks[1]->s_type,
                         psocks[1]->s_proto, &server);
      if (ret < 0)
        {
          return ret;
        }

      pserver = &server;
    }
#endif /* CONFIG_NET_TCP */

  ret = psock_bind(pserver, &addr[1].addr, len);
  if (ret < 0)
    {
      goto errout;
    }

  psock_getsockname(pserver, &addr[1].addr, &len);

#if defined(CONFIG_NET_UDP)
  if (psocks[0]->s_type == SOCK_DGRAM)
    {
      ret = psock_connect(psocks[0], &addr[1].addr, len);
      if (ret < 0)
        {
          goto errout;
        }

      ret = psock_connect(pserver, &addr[0].addr, len);
      if (ret < 0)
        {
          goto errout;
        }
    }
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP)
  if (psocks[0]->s_type == SOCK_STREAM)
    {
      FAR struct socket_conn_s *conn = psocks[1]->s_conn;

      ret = psock_listen(pserver, 2);
      if (ret < 0)
        {
          goto errout;
        }

      ret = psock_connect(psocks[0], &addr[1].addr, len);
      if (ret < 0)
        {
          goto errout;
        }

      /* Release the resource of psocks[1], accept will replace
       * this handle
       */

      psock_close(psocks[1]);

      ret = psock_accept(pserver, &addr[1].addr, &len, psocks[1],
                         conn->s_flags & _SF_NONBLOCK ? SOCK_NONBLOCK : 0);
    }
#endif /* CONFIG_NET_TCP */

errout:
#if defined(CONFIG_NET_TCP)
  if (pserver->s_type == SOCK_STREAM)
    {
      psock_close(pserver);
    }
#endif /* CONFIG_NET_TCP */

  return ret;
#else
  return -EOPNOTSUPP;
#endif /* CONFIG_NET_TCP || CONFIG_NET_UDP */
}

/****************************************************************************
 * Name: inet_shutdown
 *
 * Description:
 *   Performs the shutdown operation on an AF_INET or AF_INET6 socket
 *
 * Input Parameters:
 *   psock   Socket instance
 *   how     Specifies the type of shutdown
 *
 * Returned Value:
 *   0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int inet_shutdown(FAR struct socket *psock, int how)
{
  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
#ifdef NET_TCP_HAVE_STACK
        return tcp_shutdown(psock, how);
#else
        nwarn("WARNING: SOCK_STREAM support is not available in this "
              "configuration\n");
        return -EAFNOSUPPORT;
#endif /* NET_TCP_HAVE_STACK */
#endif /* CONFIG_NET_TCP */

      default:
        return -EOPNOTSUPP;
    }
}

/****************************************************************************
 * Name: inet_sendfile
 *
 * Description:
 *   The inet_sendfile() call may be used only when the INET socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
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

#ifdef CONFIG_NET_SENDFILE
static ssize_t inet_sendfile(FAR struct socket *psock,
                             FAR struct file *infile, FAR off_t *offset,
                             size_t count)
{
#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_sendfile(psock, infile, offset, count);
    }
#endif

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: inet_recvmsg
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  inet_recvmsg() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If msg_name is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen' is
 *   initialized to the size of the buffer associated with msg_name, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock   - A pointer to a NuttX-specific, internal socket structure
 *   msg     - Buffer to receive the message
 *   flags   - Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

static ssize_t inet_recvmsg(FAR struct socket *psock,
                            FAR struct msghdr *msg, int flags)
{
  ssize_t ret;

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold this address family.
   */

  if (msg->msg_name)
    {
      socklen_t minlen;

      /* Get the minimum socket length */

      switch (psock->s_domain)
        {
#ifdef CONFIG_NET_IPv4
        case PF_INET:
          {
            minlen = sizeof(struct sockaddr_in);
          }
          break;
#endif

#ifdef CONFIG_NET_IPv6
        case PF_INET6:
          {
            minlen = sizeof(struct sockaddr_in6);
          }
          break;
#endif

        default:
          DEBUGPANIC();
          return -EINVAL;
        }

      if (msg->msg_namelen < minlen)
        {
          return -EINVAL;
        }
    }

  /* Read from the network interface driver buffer.
   * Or perform the TCP/IP or UDP recv() operation.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
    case SOCK_STREAM:
      {
#ifdef NET_TCP_HAVE_STACK
        ret = psock_tcp_recvfrom(psock, msg, flags);
#else
        ret = -ENOSYS;
#endif
      }
      break;
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
    case SOCK_DGRAM:
      {
#ifdef NET_UDP_HAVE_STACK
        ret = psock_udp_recvfrom(psock, msg, flags);
#else
        ret = -ENOSYS;
#endif
      }
      break;
#endif /* CONFIG_NET_UDP */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
    case SOCK_CTRL:
      {
        nerr("ERROR:  Inappropriate socket type: %d\n", psock->s_type);
        ret = -EOPNOTSUPP;
      }
      break;
#endif

    default:
      {
        nerr("ERROR: Unsupported socket type: %d\n", psock->s_type);
        ret = -ENOSYS;
      }
      break;
    }

  return ret;
}

#endif /* NET_UDP_HAVE_STACK || NET_TCP_HAVE_STACK */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int inet_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the AF_INET/AF_INET6 address
   * types.
   */

#ifdef CONFIG_NET_TCP
  if (psock->s_type == SOCK_STREAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_TCP)))
    {
#ifdef NET_TCP_HAVE_STACK
      FAR struct tcp_conn_s *conn = psock->s_conn;
      int ret;

      /* Is this the last reference to the connection structure (there
       * could be more if the socket was dup'ed).
       */

      if (conn->crefs <= 1)
        {
          /* Yes... Clost the socket */

          ret = tcp_close(psock);
          if (ret < 0)
            {
              /* This would normally occur only if there is a timeout
               * from a lingering close.
               */

              nerr("ERROR: tcp_close failed: %d\n", ret);
              return ret;
            }
        }
      else
        {
          /* No.. Just decrement the reference count */

          conn->crefs--;
        }
#else
      nwarn("WARNING: SOCK_STREAM support is not available in this "
            "configuration\n");
      return -EAFNOSUPPORT;
#endif /* NET_TCP_HAVE_STACK */
    }
  else
#endif /* CONFIG_NET_TCP */
#ifdef CONFIG_NET_UDP
  if (psock->s_type == SOCK_DGRAM ||
      (psock->s_type == SOCK_CTRL &&
      (psock->s_proto == 0 || psock->s_proto == IPPROTO_UDP)))
    {
#ifdef NET_UDP_HAVE_STACK
      FAR struct udp_conn_s *conn = psock->s_conn;
      int ret;

      /* Is this the last reference to the connection structure (there
       * could be more if the socket was dup'ed).
       */

      if (conn->crefs <= 1)
        {
          /* Yes... Clost the socket */

          ret = udp_close(psock);
          if (ret < 0)
            {
              /* This would normally occur only if there is a timeout
               * from a lingering close.
               */

              nerr("ERROR: udp_close failed: %d\n", ret);
              return ret;
            }
        }
      else
        {
          /* No.. Just decrement the reference count */

          conn->crefs--;
        }
#else
      nwarn("WARNING: SOCK_DGRAM support is not available in this "
            "configuration\n");
      return -EAFNOSUPPORT;
#endif /* NET_UDP_HAVE_STACK */
    }
  else
#endif /* CONFIG_NET_UDP */
    {
      return -EBADF;
    }

  return OK;
}

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
inet_sockif(sa_family_t family, int type, int protocol)
{
  DEBUGASSERT(family == PF_INET || family == PF_INET6);

#if defined(HAVE_PFINET_SOCKETS) && defined(CONFIG_NET_ICMP_SOCKET)
  /* PF_INET, ICMP data gram sockets are a special case of raw sockets */

  if (family == PF_INET && (type == SOCK_DGRAM || type == SOCK_CTRL) &&
      protocol == IPPROTO_ICMP)
    {
      return &g_icmp_sockif;
    }
  else
#endif
#if defined(HAVE_PFINET6_SOCKETS) && defined(CONFIG_NET_ICMPv6_SOCKET)
  /* PF_INET, ICMP data gram sockets are a special case of raw sockets */

  if (family == PF_INET6 && (type == SOCK_DGRAM || type == SOCK_CTRL) &&
      protocol == IPPROTO_ICMP6)
    {
      return &g_icmpv6_sockif;
    }
  else
#endif
#if defined(NET_UDP_HAVE_STACK) || defined(NET_TCP_HAVE_STACK)
    {
      return &g_inet_sockif;
    }
#else
    {
      return NULL;
    }
#endif
}

#endif /* HAVE_INET_SOCKETS */
