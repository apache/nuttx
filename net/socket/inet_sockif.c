/****************************************************************************
 * net/socket/inet_sockif.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "sixlowpan/sixlowpan.h"
#include "usrsock/usrsock.h"
#include "socket/socket.h"

#ifdef HAVE_INET_SOCKETS

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        inet_setup(FAR struct socket *psock, int protocol);
static sockcaps_t inet_sockcaps(FAR struct socket *psock);
static void       inet_addref(FAR struct socket *psock);
static int        inet_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        inet_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        inet_listen(FAR struct socket *psock, int backlog);
static int        inet_accept(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                    FAR struct socket *newsock);
#ifndef CONFIG_DISABLE_POLL
static int        inet_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
#endif
static ssize_t    inet_send(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags);
static ssize_t    inet_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_inet_sockif =
{
  inet_setup,       /* si_setup */
  inet_sockcaps,    /* si_sockcaps */
  inet_addref,      /* si_addref */
  inet_bind,        /* si_bind */
  inet_getsockname, /* si_getsockname */
  inet_listen,      /* si_listen */
  inet_connect,     /* si_connect */
  inet_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  inet_poll,        /* si_poll */
#endif
  inet_send,        /* si_send */
  inet_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
#if defined(CONFIG_NET_TCP) && defined(NET_TCP_HAVE_STACK)
  inet_sendfile,    /* si_sendfile */
#else
  NULL,             /* si_sendfile */
#endif
#endif
  inet_recvfrom,    /* si_recvfrom */
  inet_close        /* si_close */
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
 * Name: usrsock_socket_setup
 *
 * Description:
 *   Special socket setup may be required by user sockets.
 *
 * Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *
 * Returned Value:
 *   0 on success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_USRSOCK
static int usrsock_socket_setup(int domain, int type, int protocol,
                                FAR struct socket *psock)
{
  switch (domain)
    {
      default:
        return OK;

      case PF_INET:
      case PF_INET6:
        {
#ifndef CONFIG_NET_USRSOCK_UDP
          if (type == SOCK_DGRAM)
            {
              return OK;
            }
#endif
#ifndef CONFIG_NET_USRSOCK_TCP
          if (type == SOCK_STREAM)
            {
              return OK;
            }
#endif
          psock->s_type = PF_UNSPEC;
          psock->s_conn = NULL;

          /* Let the user socket logic handle the setup...
           *
           * A return value of zero means that the operation was
           * successfully handled by usrsock.  A negative value means that
           * an error occurred.  The special error value -ENETDOWN means
           * that usrsock daemon is not running.  The caller should attempt
           * to open socket with kernel networking stack in this case.
           */

          return usrsock_socket(domain, type, protocol, psock);
        }
    }
}
#endif /* CONFIG_NET_USRSOCK */

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
 * Parameters:
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int inet_setup(FAR struct socket *psock, int protocol)
{
#ifdef CONFIG_NET_USRSOCK
  int ret;

  /* Handle special setup for user INET sockets */

  ret = usrsock_socket_setup(psock->s_domain, psock->s_type, protocol, psock);
  if (ret < 0)
    {
      if (ret == -ENETDOWN)
        {
          /* -ENETDOWN means that usrsock daemon is not running.  Attempt to
           * open socket with kernel networking stack.
           */

          nwarn("WARNING: usrsock daemon is not running\n");
        }
      else
        {
          return ret;
        }
    }
#endif /* CONFIG_NET_USRSOCK */

  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   *
   * Only SOCK_STREAM and SOCK_DGRAM and possible SOCK_RAW are supported.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        if (protocol != 0 && protocol != IPPROTO_TCP)
          {
            nerr("ERROR: Unsupported stream protocol: %d\n", protocol);
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
        if (protocol != 0 && protocol != IPPROTO_UDP)
          {
            nerr("ERROR: Unsupported datagram protocol: %d\n", protocol);
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
 * Parameters:
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
#ifdef CONFIG_NET_TCP_READAHEAD
        return SOCKCAP_NONBLOCKING;
#else
        return 0;
#endif
#endif

#ifdef NET_UDP_HAVE_STACK
      case SOCK_DGRAM:
#ifdef CONFIG_NET_UDP_READAHEAD
        return SOCKCAP_NONBLOCKING;
#else
        return 0;
#endif
#endif

#ifdef CONFIG_NET_USRSOCK
      case SOCK_USRSOCK_TYPE:
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
 *   Increment the refernce count on the underlying connection structure.
 *
 * Parameters:
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
  if (psock->s_type == SOCK_STREAM)
    {
      FAR struct tcp_conn_s *conn = psock->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
#ifdef NET_UDP_HAVE_STACK
  if (psock->s_type == SOCK_DGRAM)
    {
      FAR struct udp_conn_s *conn = psock->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
#ifdef CONFIG_NET_USRSOCK
  if (psock->s_type == SOCK_USRSOCK_TYPE)
    {
      FAR struct usrsock_conn_s *conn = psock->s_conn;
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
 * Parameters:
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
      return -EBADF;
    }

  /* Perform the binding depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_USRSOCK
      case SOCK_USRSOCK_TYPE:
        {
          FAR struct usrsock_conn_s *conn = psock->s_conn;

          DEBUGASSERT(conn != NULL);

          /* Perform the usrsock bind operation */

          ret = usrsock_bind(conn, addr, addrlen);
        }
        break;
#endif

#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
#ifdef NET_TCP_HAVE_STACK
          /* Bind a TCP/IP stream socket. */

          ret = tcp_bind(psock->s_conn, addr);

          /* Mark the socket bound */

          if (ret >= 0)
            {
              psock->s_flags |= _SF_BOUND;
            }
#else
          nwarn("WARNING: TCP/IP stack is not available in this configuration\n");
          return -ENOSYS;
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

          /* Mark the socket bound */

          if (ret >= 0)
            {
              psock->s_flags |= _SF_BOUND;
            }
#else
          nwarn("WARNING: UDP stack is not available in this configuration\n");
          ret = -ENOSYS;
#endif
        }
        break;
#endif /* CONFIG_NET_UDP */

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
 * Parameters:
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
#ifdef CONFIG_NET_USRSOCK
  if (psock->s_type == SOCK_USRSOCK_TYPE)
    {
      FAR struct usrsock_conn_s *conn = psock->s_conn;

      DEBUGASSERT(conn != NULL);

      /* Handle usrsock getsockname */

      return usrsock_getsockname(conn, addr, addrlen);
    }
#endif

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
 * Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See list() for the set of appropriate error values.
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
#ifdef CONFIG_NET_USRSOCK
      if (psock->s_type == SOCK_USRSOCK_TYPE)
        {
#warning "Missing logic"
        }
#endif

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
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
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
#ifdef CONFIG_NET_USRSOCK
      if (psock->s_type == SOCK_USRSOCK_TYPE)
        {
#warning "Missing logic"
        }
#endif

      nerr("ERROR:  Inappropreat socket type: %d\n", psock->s_type);
      return -EOPNOTSUPP;
    }

  /* Verify that a valid memory block has been provided to receive the
   * address
   */

  if (addr != NULL)
    {
      /* If an address is provided, then the length must also be provided. */

      DEBUGASSERT(addrlen > 0);

      /* A valid length depends on the address domain */

      switch (psock->s_domain)
        {
#ifdef CONFIG_NET_IPv4
        case PF_INET:
          {
            if (*addrlen < sizeof(struct sockaddr_in))
              {
                return -EBADF;
              }
          }
          break;
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
        case PF_INET6:
          {
            if (*addrlen < sizeof(struct sockaddr_in6))
              {
                return -EBADF;
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
  /* Perform the local accept operation (with the network locked) */

  net_lock();
  ret = psock_tcp_accept(psock, addr, addrlen, &newsock->s_conn);
  if (ret < 0)
    {
      nerr("ERROR: psock_tcp_accept failed: %d\n", ret);
      goto errout_with_lock;
    }

   /* Begin monitoring for TCP connection events on the newly connected
    * socket
    */

  ret = net_startmonitor(newsock);
  if (ret < 0)
    {
      /* net_startmonitor() can only fail on certain race conditions where
       * the connection was lost just before this function was called.  Undo
       * everything we have done and return a failure.
       */

      goto errout_after_accept;
    }

  net_unlock();
  return OK;

errout_after_accept:
  psock_close(newsock);

errout_with_lock:
  net_unlock();
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

#if defined(HAVE_TCP_POLL) || defined(HAVE_UDP_POLL)
static inline int inet_pollsetup(FAR struct socket *psock,
                                 FAR struct pollfd *fds)
{
#ifdef HAVE_TCP_POLL
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollsetup(psock, fds);
    }
  else
#endif /* HAVE_TCP_POLL */
#ifdef HAVE_UDP_POLL
  if (psock->s_type != SOCK_STREAM)
    {
      return udp_pollsetup(psock, fds);
    }
  else
#endif /* HAVE_UDP_POLL */
    {
      return -ENOSYS;
    }
}
#endif /* HAVE_TCP_POLL || HAVE_UDP_POLL */

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

#if defined(HAVE_TCP_POLL) || defined(HAVE_UDP_POLL)
static inline int inet_pollteardown(FAR struct socket *psock,
                                    FAR struct pollfd *fds)
{
#ifdef HAVE_TCP_POLL
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollteardown(psock, fds);
    }
  else
#endif /* HAVE_TCP_POLL */
#ifdef HAVE_UDP_POLL
  if (psock->s_type == SOCK_DGRAM)
    {
      return udp_pollteardown(psock, fds);
    }
  else
#endif /* HAVE_UDP_POLL */
    {
      return -ENOSYS;
    }
}
#endif /* HAVE_TCP_POLL || HAVE_UDP_POLL */

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

#ifndef CONFIG_DISABLE_POLL
static int inet_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                     bool setup)
{
#ifdef CONFIG_NET_USRSOCK
  if (psock->s_type == SOCK_USRSOCK_TYPE)
    {
      /* Perform usrsock setup/teardown. */

      return usrsock_poll(psock, fds, setup);
    }
  else
#endif
#if defined(HAVE_TCP_POLL) || defined(HAVE_UDP_POLL)

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
#endif /* HAVE_TCP_POLL || !HAVE_UDP_POLL */
}
#endif /* !CONFIG_DISABLE_POLL */

/****************************************************************************
 * Name: inet_send
 *
 * Description:
 *   The inet_send() call may be used only when the socket is in a connected
 *   state  (so that the intended recipient is known).
 *
 * Parameters:
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
  ssize_t ret;

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
#ifdef CONFIG_NET_6LOWPAN
          /* Try 6LoWPAN TCP packet send */

          ret = psock_6lowpan_tcp_send(psock, buf, len);

#if defined(CONFIG_NETDEV_MULTINIC) && defined(NET_TCP_HAVE_STACK)
          if (ret < 0)
            {
              /* TCP/IP packet send */

              ret = psock_tcp_send(psock, buf, len);
            }
#endif /* CONFIG_NETDEV_MULTINIC && NET_TCP_HAVE_STACK */
#elif defined(NET_TCP_HAVE_STACK)
          ret = psock_tcp_send(psock, buf, len);
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

#if defined(CONFIG_NETDEV_MULTINIC) && defined(NET_UDP_HAVE_STACK)
          if (ret < 0)
            {
              /* UDP/IP packet send */

              ret = psock_udp_send(psock, buf, len);
            }
#endif /* CONFIG_NETDEV_MULTINIC && NET_UDP_HAVE_STACK */
#elif defined(NET_UDP_HAVE_STACK)
          /* Only UDP/IP packet send */

          ret = psock_udp_send(psock, buf, len);
#else
          ret = -ENOSYS;
#endif /* CONFIG_NET_6LOWPAN */
        }
        break;
#endif /* CONFIG_NET_UDP */

  /* Special case user sockets */

#ifdef CONFIG_NET_USRSOCK
      case SOCK_USRSOCK_TYPE:
        {
          ret = usrsock_sendto(psock, buf, len, NULL, 0);
        }
        break;
#endif /*CONFIG_NET_USRSOCK*/

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
 * Parameters:
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
                           size_t len, int flags, FAR const struct sockaddr *to,
                           socklen_t tolen)
{
  socklen_t minlen;
  ssize_t nsent;

#ifdef CONFIG_NET_USRSOCK
  if (psock->s_type == SOCK_USRSOCK_TYPE)
    {
      /* Perform the usrsock sendto operation */

      nsent = usrsock_sendto(psock, buf, len, to, tolen);
    }
  else
#endif
    {
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

      nsent = psock_6lowpan_udp_sendto(psock, buf, len, flags, to, tolen);

#if defined(CONFIG_NETDEV_MULTINIC) && defined(NET_UDP_HAVE_STACK)
      if (nsent < 0)
        {
          /* UDP/IP packet sendto */

          nsent = psock_udp_sendto(psock, buf, len, flags, to, tolen);
        }
#endif /* CONFIG_NETDEV_MULTINIC && NET_UDP_HAVE_STACK */
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
    }

  return nsent;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

#endif /* HAVE_INET_SOCKETS */
