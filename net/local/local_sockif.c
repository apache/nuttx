/****************************************************************************
 * net/local/local_sockif.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/net.h>
#include <socket/socket.h>

#include "local/local.h"

#ifdef CONFIG_NET_LOCAL

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        local_setup(FAR struct socket *psock, int protocol);
static sockcaps_t local_sockcaps(FAR struct socket *psock);
static void       local_addref(FAR struct socket *psock);
static int        local_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        local_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        local_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
#ifndef CONFIG_NET_LOCAL_STREAM
static int        local_listen(FAR struct socket *psock, int backlog);
#endif
static int        local_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
#ifndef CONFIG_NET_LOCAL_STREAM
static int        local_accept(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                    FAR struct socket *newsock);
#endif
#ifndef CONFIG_DISABLE_POLL
static int        local_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
#endif
static ssize_t    local_send(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags);
static ssize_t    local_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen);
static int        local_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_local_sockif =
{
  local_setup,       /* si_setup */
  local_sockcaps,    /* si_sockcaps */
  local_addref,      /* si_addref */
  local_bind,        /* si_bind */
  local_getsockname, /* si_getsockname */
  local_getpeername, /* si_getpeername */
  local_listen,      /* si_listen */
  local_connect,     /* si_connect */
  local_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  local_poll,        /* si_poll */
#endif
  local_send,        /* si_send */
  local_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,              /* si_sendfile */
#endif
  local_recvfrom,    /* si_recvfrom */
  local_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_sockif_alloc
 *
 * Description:
 *   Allocate and attach a local, Unix domain connection structure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
static int local_sockif_alloc(FAR struct socket *psock)
{
  /* Allocate the local connection structure */

  FAR struct local_conn_s *conn = local_alloc();
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->lc_crefs == 0);
  conn->lc_crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif

/****************************************************************************
 * Name: local_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int local_setup(FAR struct socket *psock, int protocol)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   *
   * REVIST:  Only SOCK_STREAM and SOCK_DGRAM are supported.  Should also
   * support SOCK_RAW.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
        if (protocol != 0 && protocol != IPPROTO_TCP)
          {
            return -EPROTONOSUPPORT;
          }

        /* Allocate and attach the local connection structure */

        return local_sockif_alloc(psock);
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        if (protocol != 0 && protocol != IPPROTO_UDP)
          {
            return -EPROTONOSUPPORT;
          }

        /* Allocate and attach the local connection structure */

        return local_sockif_alloc(psock);
#endif /* CONFIG_NET_LOCAL_DGRAM */

      default:
        return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: local_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t local_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: local_addref
 *
 * Description:
 *   Increment the refernce count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void local_addref(FAR struct socket *psock)
{
  FAR struct local_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              psock->s_domain == PF_LOCAL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->lc_crefs > 0 && conn->lc_crefs < 255);
  conn->lc_crefs++;
}

/****************************************************************************
 * Name: local_bind
 *
 * Description:
 *   local_bind() gives the socket 'psock' the local address 'addr'.  'addr'
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

static int local_bind(FAR struct socket *psock,
                      FAR const struct sockaddr *addr, socklen_t addrlen)
{
  int ret;

  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_LOCAL || addrlen < sizeof(sa_family_t))
    {
      nerr("ERROR: Invalid address length: %d < %d\n",
           addrlen, sizeof(sa_family_t));
      return -EBADF;
    }

  /* Perform the binding depending on the protocol type */

  switch (psock->s_type)
    {
      /* Bind a local TCP/IP stream or datagram socket  */

#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
#endif
#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
#endif
        {
          /* Bind the Unix domain connection structure */

          ret = psock_local_bind(psock, addr, addrlen);

          /* Mark the socket bound */

          if (ret >= 0)
            {
              psock->s_flags |= _SF_BOUND;
            }
        }
        break;
#endif /* CONFIG_NET_LOCAL_STREAM || CONFIG_NET_LOCAL_DGRAM */

      default:
        ret = -EBADF;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: local_getsockname
 *
 * Description:
 *   The local_getsockname() function retrieves the locally-bound name of
 *   the specified local socket, stores this address in the sockaddr
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

static int local_getsockname(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  FAR struct sockaddr_un *unaddr = (FAR struct sockaddr_un *)addr;
  FAR struct local_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              unaddr != NULL && addrlen != NULL &&
              *addrlen >= sizeof(sa_family_t));

  if (*addrlen < sizeof(sa_family_t))
    {
      /* This is apparently not an error */

      *addrlen = 0;
      return OK;
    }

  conn = (FAR struct local_conn_s *)psock->s_conn;

  /* Save the address family */

  unaddr->sun_family = AF_LOCAL;
  if (*addrlen > sizeof(sa_family_t))
    {
      /* Now copy the address description.  */

      if (conn->lc_type == LOCAL_TYPE_UNNAMED)
        {
          /* Zero-length sun_path... This is an abstract Unix domain socket */

          *addrlen = sizeof(sa_family_t);
        }
      else /* conn->lctype = LOCAL_TYPE_PATHNAME */
        {
          /* Get the full length of the socket name (including null terminator) */

          int namelen = strlen(conn->lc_path) + 1;

          /* Get the available length in the user-provided buffer. */

          int pathlen = *addrlen - sizeof(sa_family_t);

          /* Clip the socket name size so that if fits in the user buffer */

          if (pathlen < namelen)
            {
              namelen = pathlen;
            }

          /* Copy the path into the user address structure */

          (void)strncpy(unaddr->sun_path, conn->lc_path, namelen);
          unaddr->sun_path[pathlen - 1] = '\0';

          *addrlen = sizeof(sa_family_t) + namelen;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: local_getpeername
 *
 * Description:
 *   The local_getpeername() function retrieves the remote-connected name of
 *   the specified local socket, stores this address in the sockaddr
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

static int local_getpeername(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  return local_getsockname(psock, addr, addrlen);
}

/****************************************************************************
 * Name: local_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of local
 *   unix sockets, psock_listen() calls this function.  The psock_listen()
 *   call applies only to sockets of type SOCK_STREAM or SOCK_SEQPACKET.
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
 *   returned.  See list() for the set of appropriate error values.
 *
 ****************************************************************************/

#ifndef CONFIG_NET_LOCAL_STREAM
int local_listen(FAR struct socket *psock, int backlog)
{
  return -EOPNOTSUPP;
}
#endif

/****************************************************************************
 * Name: local_connect
 *
 * Description:
 *   local_connect() connects the local socket referred to by the structure
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
 *   local_connect() only once; connectionless protocol sockets may use
 *   local_connect() multiple times to change their association.
 *   Connectionless sockets may dissolve the association by connecting to
 *   an address with the sa_family member of sockaddr set to AF_UNSPEC.
 *
 * Input Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failue.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int local_connect(FAR struct socket *psock,
                         FAR const struct sockaddr *addr, socklen_t addrlen)
{
  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_LOCAL || addrlen < sizeof(sa_family_t))
    {
      return -EBADF;
    }

  /* Perform the connection depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
        {
          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(psock->s_flags))
            {
              return -EISCONN;
            }

          /* It's not...  Connect to the local Unix domain server */

          return psock_local_connect(psock, addr);
        }
        break;
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        {
          /* Perform the datagram connection logic */
#warning Missing logic
          return -ENOSYS;
        }
        break;
#endif /* CONFIG_NET_LOCAL_DGRAM */

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Name: local_accept
 *
 * Description:
 *   The pkt_accept function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an pkt_accept.
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
 *   not marked as non-blocking, pkt_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, pkt_accept returns
 *   EAGAIN.
 *
 * Input Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifndef CONFIG_NET_LOCAL_STREAM
static int local_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                        FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  return -EAFNOSUPPORT;
}
#endif

/****************************************************************************
 * Name: local_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to local_poll which, indiectly, calls to function.
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
static int local_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                      bool setup)
{
#ifndef HAVE_LOCAL_POLL
  return -ENOSYS;
#else
  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the TCP/IP poll() setup */

      return local_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the TCP/IP poll() teardown */

      return local_pollteardown(psock, fds);
    }
#endif /* HAVE_LOCAL_POLL */
}
#endif /* !CONFIG_DISABLE_POLL */

/****************************************************************************
 * Name: local_send
 *
 * Description:
 *   Implements the send() operation for the case of the local, Unix socket.
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

static ssize_t local_send(FAR struct socket *psock, FAR const void *buf,
                          size_t len, int flags)
{
  ssize_t ret;

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
        {
          /* Local TCP packet send */

          ret = psock_local_send(psock, buf, len, flags);
        }
        break;
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        {
          /* Local UDP packet send */
#warning Missing logic
          ret = -ENOSYS;
        }
        break;
#endif /* CONFIG_NET_LOCAL_DGRAM */

      default:
        {
          /* EDESTADDRREQ.  Signifies that the socket is not connection-mode
           * and no peer address is set.
           */

          ret = -EDESTADDRREQ;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: local_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the local, Unix socket.
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

ssize_t local_sendto(FAR struct socket *psock, FAR const void *buf,
                     size_t len, int flags, FAR const struct sockaddr *to,
                     socklen_t tolen)
{
  ssize_t nsent;

  /* Verify that a valid address has been provided */

  if (to->sa_family != AF_LOCAL || tolen < sizeof(sa_family_t))
    {
      nerr("ERROR: Unrecognized address family: %d\n",
           to->sa_family);
      return -EAFNOSUPPORT;
    }

#ifdef CONFIG_NET_LOCAL_DGRAM
  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR: Connected socket\n");
      return -EISCONN;
    }

  /* Now handle the local UDP sendto() operation */

  nsent = psock_local_sendto(psock, buf, len, flags, to, tolen);
#else
  nsent = -EISCONN;
#endif /* CONFIG_NET_LOCAL_DGRAM */

  return nsent;
}

/****************************************************************************
 * Name: local_close
 *
 * Description:
 *   Performs the close operation on a local, Unix socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; a negated errno value is returned on any failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int local_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the local address type */

  switch (psock->s_type)
    {
#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
#endif
#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
#endif
        {
          FAR struct local_conn_s *conn = psock->s_conn;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->lc_crefs <= 1)
            {
              conn->lc_crefs = 0;
              local_release(conn);
            }
          else
           {
             /* No.. Just decrement the reference count */

             conn->lc_crefs--;
           }

          return OK;
        }
#endif /* CONFIG_NET_LOCAL_STREAM || CONFIG_NET_LOCAL_DGRAM */

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_LOCAL */
