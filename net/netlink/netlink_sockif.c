/****************************************************************************
 * net/netlink/netlink_sockif.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "netlink/netlink.h"

#ifdef CONFIG_NET_NETLINK

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  netlink_setup(FAR struct socket *psock, int protocol);
static sockcaps_t netlink_sockcaps(FAR struct socket *psock);
static void netlink_addref(FAR struct socket *psock);
static int  netlink_bind(FAR struct socket *psock,
              FAR const struct sockaddr *addr, socklen_t addrlen);
static int  netlink_getsockname(FAR struct socket *psock,
              FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int  netlink_getpeername(FAR struct socket *psock,
              FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int  netlink_listen(FAR struct socket *psock, int backlog);
static int  netlink_connect(FAR struct socket *psock,
              FAR const struct sockaddr *addr, socklen_t addrlen);
static int  netlink_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
              FAR socklen_t *addrlen, FAR struct socket *newsock);
static int  netlink_poll(FAR struct socket *psock, FAR struct pollfd *fds,
              bool setup);
static ssize_t netlink_send(FAR struct socket *psock,
              FAR const void *buf, size_t len, int flags);
static ssize_t netlink_sendto(FAR struct socket *psock, FAR const void *buf,
              size_t len, int flags, FAR const struct sockaddr *to,
              socklen_t tolen);
static ssize_t netlink_recvfrom(FAR struct socket *psock, FAR void *buf,
              size_t len, int flags, FAR struct sockaddr *from,
              FAR socklen_t *fromlen);
static int netlink_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_netlink_sockif =
{
  netlink_setup,        /* si_setup */
  netlink_sockcaps,     /* si_sockcaps */
  netlink_addref,       /* si_addref */
  netlink_bind,         /* si_bind */
  netlink_getsockname,  /* si_getsockname */
  netlink_getpeername,  /* si_getpeername */
  netlink_listen,       /* si_listen */
  netlink_connect,      /* si_connect */
  netlink_accept,       /* si_accept */
  netlink_poll,         /* si_poll */
  netlink_send,         /* si_send */
  netlink_sendto,       /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,                 /* si_sendfile */
#endif
  netlink_recvfrom,     /* si_recvfrom */
  netlink_close         /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    - A pointer to a user allocated socket structure to be
 *              initialized.
 *   protocol - NetLink socket protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int netlink_setup(FAR struct socket *psock, int protocol)
{
  int domain = psock->s_domain;
  int type = psock->s_type;

  /* Verify that the protocol is supported */

  DEBUGASSERT((unsigned int)protocol <= UINT8_MAX);

  switch (protocol)
    {
#ifdef CONFIG_NETLINK_ROUTE
      case NETLINK_ROUTE:
        break;
#endif

      default:
        return -EPROTONOSUPPORT;
    }

  /* Verify the socket type (domain should always be PF_NETLINK here) */

  if (domain == PF_NETLINK && (type == SOCK_RAW || type == SOCK_DGRAM))
    {
      /* Allocate the NetLink socket connection structure and save it in the
       * new socket instance.
       */

      FAR struct netlink_conn_s *conn = netlink_alloc();
      if (conn == NULL)
        {
          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

      /* Initialize the connection instance */

      conn->protocol = (uint8_t)protocol;

      /* Set the reference count on the connection structure.  This
       * reference count will be incremented only if the socket is
       * dup'ed
       */

      conn->crefs = 1;

      /* Attach the connection instance to the socket */

      psock->s_conn = conn;
      return OK;
    }

  return -EPROTONOSUPPORT;
}

/****************************************************************************
 * Name: netlink_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket capabilities is returned.
 *
 ****************************************************************************/

static sockcaps_t netlink_sockcaps(FAR struct socket *psock)
{
  return 0;
}

/****************************************************************************
 * Name: netlink_addref
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

static void netlink_addref(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: netlink_bind
 *
 * Description:
 *   netlink_bind() gives the socket 'conn' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   conn     NetLink socket connection structure
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int netlink_bind(FAR struct socket *psock,
                        FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct sockaddr_nl *nladdr;
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen >= sizeof(struct sockaddr_nl));

  /* Save the address information in the connection structure */

  nladdr          = (FAR struct sockaddr_nl *)addr;
  conn            = (FAR struct netlink_conn_s *)psock->s_conn;

  conn->pid       = nladdr->nl_pid;
  conn->groups    = nladdr->nl_groups;

  psock->s_flags |= _SF_BOUND;
  return OK;
}

/****************************************************************************
 * Name: netlink_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   conn     NetLink socket connection structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

static int netlink_getsockname(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen)
{
  FAR struct sockaddr_nl *nladdr;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen != NULL && *addrlen >= sizeof(struct sockaddr_nl));

  /* Return the address information in the address structure */

  nladdr = (FAR struct sockaddr_nl *)addr;
  memset(nladdr, 0, sizeof(struct sockaddr_nl));

  nladdr->nl_family     = AF_NETLINK;

  if (_SS_ISBOUND(psock->s_flags))
    {
      FAR struct netlink_conn_s *conn;

      conn              = (FAR struct netlink_conn_s *)psock->s_conn;
      nladdr->nl_pid    = conn->pid;
      nladdr->nl_groups = conn->groups;
    }

  *addrlen = sizeof(struct sockaddr_nl);
  return OK;
}

/****************************************************************************
 * Name: netlink_getpeername
 *
 * Description:
 *   The netlink_getpeername() function retrieves the remote-connected name
 *   of the specified packet socket, stores this address in the sockaddr
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

static int netlink_getpeername(FAR struct socket *psock,
                               FAR struct sockaddr *addr,
                               FAR socklen_t *addrlen)
{
#warning Missing logic for NETLINK getsockname
  return -EOPNOTSUPP;  /* Or maybe return -EAFNOSUPPORT; */
}

/****************************************************************************
 * Name: netlink_listen
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
 *   psock    Reference to an internal, bound socket structure.
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

static int netlink_listen(FAR struct socket *psock, int backlog)
{
#warning Missing logic for NETLINK listen
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_connect
 *
 * Description:
 *   Perform a netlink connection
 *
 * Input Parameters:
 *   psock   A reference to the socket structure of the socket to be connected
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int netlink_connect(FAR struct socket *psock,
                           FAR const struct sockaddr *addr,
                           socklen_t addrlen)
{
#warning Missing logic for NETLINK connect
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_accept
 *
 * Description:
 *   The netlink_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
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
 *   addrlen  Input:  Allocated size of 'addr'
 *            Return: Actual size returned size of 'addr'
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

static int netlink_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                          FAR socklen_t *addrlen, FAR struct socket *newsock)
{
#warning Missing logic for NETLINK accept
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Tear down the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int netlink_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                        bool setup)
{
#warning Missing logic for NETLINK poll
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_send
 *
 * Description:
 *   The netlink_send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t netlink_send(FAR struct socket *psock, FAR const void *buf,
                            size_t len, int flags)
{
  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);

  /* The socket must be connected in order to use send */

  if (_SS_ISBOUND(psock->s_flags))
    {
      FAR struct netlink_conn_s *conn;
      struct sockaddr_nl nladdr;

      /* Get the underlying connection structure */

      conn             = (FAR struct netlink_conn_s *)psock->s_conn;

      /* Format the address */

      nladdr.nl_family = AF_NETLINK;
      nladdr.nl_pad    = 0;
      nladdr.nl_pid    = conn->pid;
      nladdr.nl_groups = conn->groups;

      /* Then let sendto() perform the actual send operation */

      return netlink_sendto(psock, buf, len, flags,
                            (FAR const struct sockaddr *)&nladdr,
                            sizeof(struct sockaddr_nl));
    }

  /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and no
   * peer address is set.
   */

  return -EDESTADDRREQ;
}

/****************************************************************************
 * Name: netlink_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket to be connected
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t netlink_sendto(FAR struct socket *psock, FAR const void *buf,
                              size_t len, int flags,
                              FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct netlink_conn_s *conn;
  FAR struct nlmsghdr *nlmsg;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL &&
              to != NULL && tolen >= sizeof(struct sockaddr_nl));

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Get a reference to the netlink message */

  nlmsg = (FAR struct nlmsghdr *)buf;
  DEBUGASSERT(nlmsg->nlmsg_len >= sizeof(struct nlmsghdr));

  switch (conn->protocol)
    {
#ifdef CONFIG_NETLINK_ROUTE
      case NETLINK_ROUTE:
        ret = netlink_route_sendto(psock, nlmsg, len, flags,
                                   (FAR struct sockaddr_nl *)to,
                                   tolen);
        break;
#endif

      default:
       ret= -EOPNOTSUPP;
       break;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'fromlen'
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags (ignored)
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 ****************************************************************************/

static ssize_t netlink_recvfrom(FAR struct socket *psock, FAR void *buf,
                                size_t len, int flags,
                                FAR struct sockaddr *from,
                                FAR socklen_t *fromlen)
{
  FAR struct netlink_conn_s *conn;
  FAR struct nlmsghdr *nlmsg;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);
  DEBUGASSERT(from == NULL ||
              (fromlen != NULL && *fromlen >= sizeof(struct sockaddr_nl)));

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Get a reference to the NetLink message */

  nlmsg = (FAR struct nlmsghdr *)buf;

  switch (conn->protocol)
    {
#ifdef CONFIG_NETLINK_ROUTE
      case NETLINK_ROUTE:
        ret = netlink_route_recvfrom(psock, nlmsg, len, flags,
                                     (FAR struct sockaddr_nl *)from);
        if (ret >= 0 && fromlen != NULL)
          {
            *fromlen = sizeof(struct sockaddr_nl);
          }

        break;
#endif

      default:
       ret= -EOPNOTSUPP;
       break;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_close
 *
 * Description:
 *   Performs the close operation on a NetLink socket instance
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

static int netlink_close(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn = psock->s_conn;
  int ret = OK;

  /* Perform some pre-close operations for the NETLINK socket type. */

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform user-space daemon of socket close. */

#warning Missing logic

      /* Free the connection structure */

      conn->crefs = 0;
      netlink_free(psock->s_conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: netlink_close failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return ret;
}

#endif /* CONFIG_NET_NETLINK */
