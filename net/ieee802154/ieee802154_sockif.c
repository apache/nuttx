/****************************************************************************
 * net/socket/ieee802154_sockif.c
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

#include <socket/socket.h>
#include <netpacket/ieee802154.h>

#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>

#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        ieee802154_setup(FAR struct socket *psock, int protocol);
static sockcaps_t ieee802154_sockcaps(FAR struct socket *psock);
static void       ieee802154_addref(FAR struct socket *psock);
static int        ieee802154_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        ieee802154_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        ieee802154_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        ieee802154_listen(FAR struct socket *psock, int backlog);
static int        ieee802154_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen);
static int        ieee802154_accept(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                    FAR struct socket *newsock);
#ifndef CONFIG_DISABLE_POLL
static int        ieee802154_poll_local(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
#endif
static ssize_t    ieee802154_send(FAR struct socket *psock,
                   FAR const void *buf, size_t len, int flags);
static ssize_t    ieee802154_sendto(FAR struct socket *psock,
                   FAR const void *buf, size_t len, int flags,
                   FAR const struct sockaddr *to, socklen_t tolen);
static int        ieee802154_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_ieee802154_sockif =
{
  ieee802154_setup,       /* si_setup */
  ieee802154_sockcaps,    /* si_sockcaps */
  ieee802154_addref,      /* si_addref */
  ieee802154_bind,        /* si_bind */
  ieee802154_getsockname, /* si_getsockname */
  ieee802154_getpeername, /* si_getpeername */
  ieee802154_listen,      /* si_listen */
  ieee802154_connect,     /* si_connect */
  ieee802154_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  ieee802154_poll_local,  /* si_poll */
#endif
  ieee802154_send,        /* si_send */
  ieee802154_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,                   /* si_sendfile */
#endif
  ieee802154_recvfrom,    /* si_recvfrom */
  ieee802154_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_sockif_alloc
 *
 * Description:
 *   Allocate and attach a PF_IEEE802154 connection structure.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static int ieee802154_sockif_alloc(FAR struct socket *psock)
{
  /* Allocate the packet socket connection structure and save in the new
   * socket instance.
   */

  FAR struct ieee802154_conn_s *conn = ieee802154_conn_alloc();
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

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

/****************************************************************************
 * Name: ieee802154_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int ieee802154_setup(FAR struct socket *psock, int protocol)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   *
   * Only SOCK_DGRAM is supported (since the MAC header is stripped)
   */

  if (psock->s_type == SOCK_DGRAM)
    {
      return ieee802154_sockif_alloc(psock);
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: ieee802154_sockcaps
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

static sockcaps_t ieee802154_sockcaps(FAR struct socket *psock)
{
  return 0;
}

/****************************************************************************
 * Name: ieee802154_addref
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

static void ieee802154_addref(FAR struct socket *psock)
{
  FAR struct ieee802154_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              psock->s_type == SOCK_DGRAM);

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: ieee802154_connect
 *
 * Description:
 *   ieee802154_connect() connects the local socket referred to by the structure
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
 *   ieee802154_connect() only once; connectionless protocol sockets may use
 *   ieee802154_connect() multiple times to change their association.
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

static int ieee802154_connect(FAR struct socket *psock,
                              FAR const struct sockaddr *addr,
                              socklen_t addrlen)
{
  FAR struct ieee802154_conn_s *conn;
  FAR struct sockaddr_ieee802154_s *ieeeaddr;
  int ret;

  DEBUGASSERT(psock != NULL || addr != NULL);
  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Verify the address family */

  if (addr->sa_family == AF_IEEE802154)
    {
      /* Save the "connection" address */

      ieeeaddr = (FAR struct sockaddr_ieee802154_s *)addr;
      memcpy(&conn->raddr, &ieeeaddr->sa_addr,
             sizeof(struct ieee802154_saddr_s));

      /* Mark the socket as connected. */

      psock->s_flags |= _SF_CONNECTED;
      ret = OK;
    }
  else
    {
      /* The specified address is not a valid address for the address family
       * of the specified socket.
       */

      ret = -EAFNOSUPPORT;
    }

  return ret;
}

/****************************************************************************
 * Name: ieee802154_accept
 *
 * Description:
 *   The ieee802154_accept function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an ieee802154_accept.
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
 *   not marked as non-blocking, ieee802154_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, ieee802154_accept returns
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

static int ieee802154_accept(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen,
                             FAR struct socket *newsock)
{
  return -EAFNOSUPPORT;
}

/****************************************************************************
 * Name: ieee802154_bind
 *
 * Description:
 *   ieee802154_bind() gives the socket 'psock' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long.  Traditionally, this is called
 *   "assigning a name to a socket."  When a socket is created with
 *   socket(), it exists in a name space (address family) but has no name
 *   assigned.
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

static int ieee802154_bind(FAR struct socket *psock,
                           FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR const struct sockaddr_ieee802154_s *iaddr;
  FAR struct radio_driver_s *radio;
  FAR struct ieee802154_conn_s *conn;

  DEBUGASSERT(psock != NULL && addr != NULL);

  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_IEEE802154 ||
      addrlen < sizeof(struct sockaddr_ieee802154_s))
    {
      nerr("ERROR: Invalid family: %u or address length: %d < %d\n",
           addr->sa_family, addrlen, sizeof(struct sockaddr_ieee802154_s));
      return -EBADF;
    }

  /* Bind a PF_IEEE802154 socket to an network device. */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR: Invalid socket type: %u\n", psock->s_type);
      return -EBADF;
    }

  /* Verify that the socket is not already bound. */

  if (_SS_ISBOUND(psock->s_flags))
    {
      nerr("ERROR: Already bound\n");
      return -EINVAL;
    }

  iaddr = (FAR const struct sockaddr_ieee802154_s *)addr;

  /* Very that some address was provided */
  /* REVISIT: Currently and explict address must be assigned.  Should we
   * support some moral equivalent to INADDR_ANY?
   */

  if (iaddr->sa_addr.s_mode == IEEE802154_ADDRMODE_NONE)
    {
      nerr("ERROR: No address provided\n");
      return -EADDRNOTAVAIL;
    }

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;

  /* Find the device associated with the requested address */

  radio = ieee802154_find_device(conn, &iaddr->sa_addr);
  if (radio == NULL)
    {
      nerr("ERROR: No radio at this address\n");
      return -ENODEV;
    }

  /* Save the address as the socket's local address */

  memcpy(&conn->laddr, &iaddr->sa_addr, sizeof(struct ieee802154_saddr_s));

  /* Mark the socket bound */

  psock->s_flags |= _SF_BOUND;
  return OK;
}

/****************************************************************************
 * Name: ieee802154_getsockname
 *
 * Description:
 *   The ieee802154_getsockname() function retrieves the locally-bound name of the
 *   specified packet socket, stores this address in the sockaddr structure
 *   pointed to by the 'addr' argument, and stores the length of this
 *   address in the object pointed to by the 'addrlen' argument.
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

static int ieee802154_getsockname(FAR struct socket *psock,
                                  FAR struct sockaddr *addr, FAR
                                  socklen_t *addrlen)
{
  FAR struct ieee802154_conn_s *conn;
  FAR struct sockaddr_ieee802154_s tmp;
  socklen_t copylen;

  DEBUGASSERT(psock != NULL && addr != NULL && addrlen != NULL);

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Create a copy of the full address on the stack */

  tmp.sa_family = AF_IEEE802154;
  memcpy(&tmp.sa_addr, &conn->laddr, sizeof(struct ieee802154_saddr_s));

  /* Copy to the user buffer, truncating if necessary */

  copylen = sizeof(struct sockaddr_ieee802154_s);
  if (copylen > *addrlen)
    {
      copylen = *addrlen;
    }

  memcpy(addr, &tmp, copylen);

  /* Return the actual size transferred */

  *addrlen = copylen;
  return OK;
}

/****************************************************************************
 * Name: ieee802154_getpeername
 *
 * Description:
 *   The ieee802154_getpeername() function retrieves the remote-connectd name of the
 *   specified packet socket, stores this address in the sockaddr structure
 *   pointed to by the 'addr' argument, and stores the length of this
 *   address in the object pointed to by the 'addrlen' argument.
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

static int ieee802154_getpeername(FAR struct socket *psock,
                                  FAR struct sockaddr *addr, FAR
                                  socklen_t *addrlen)
{
  FAR struct ieee802154_conn_s *conn;
  FAR struct sockaddr_ieee802154_s tmp;
  socklen_t copylen;

  DEBUGASSERT(psock != NULL && addr != NULL && addrlen != NULL);

  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Create a copy of the full address on the stack */

  tmp.sa_family = PF_IEEE802154;
  memcpy(&tmp.sa_addr, &conn->raddr, sizeof(struct ieee802154_saddr_s));

  /* Copy to the user buffer, truncating if necessary */

  copylen = sizeof(struct sockaddr_ieee802154_s);
  if (copylen > *addrlen)
    {
      copylen = *addrlen;
    }

  memcpy(addr, &tmp, copylen);

  /* Return the actual size transferred */

  *addrlen = copylen;
  return OK;
}

/****************************************************************************
 * Name: ieee802154_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of
 *   PF_IEEE802154 sockets, psock_listen() calls this function.  The listen()
 *   call does not apply only to PF_IEEE802154 sockets.
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

int ieee802154_listen(FAR struct socket *psock, int backlog)
{
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: ieee802154_poll
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
static int ieee802154_poll_local(FAR struct socket *psock,
                                 FAR struct pollfd *fds, bool setup)
{
  /* We should need to support some kind of write ahead buffering for this
   * feature.
   */

#warning Missing logic
  return -ENOSYS;
}
#endif /* !CONFIG_DISABLE_POLL */

/****************************************************************************
 * Name: ieee802154_send
 *
 * Description:
 *   Socket send() method for the PF_IEEE802154 socket.
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

static ssize_t ieee802154_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len, int flags)
{
  struct sockaddr_ieee802154_s to;
  FAR struct ieee802154_conn_s *conn;
  ssize_t ret;

  DEBUGASSERT(psock != NULL || buf != NULL);
  conn = (FAR struct ieee802154_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Only SOCK_DGRAM is supported (because the MAC header is stripped) */

  if (psock->s_type == SOCK_DGRAM)
    {
      /* send() may be used only if the socket has been connected. */

      if (!_SS_ISCONNECTED( psock->s_flags) ||
          conn->raddr.s_mode == IEEE802154_ADDRMODE_NONE)
        {
          ret = -ENOTCONN;
        }
      else
        {
          to.sa_family = AF_IEEE802154;
          memcpy(&to.sa_addr, &conn->raddr,
                 sizeof(struct ieee802154_saddr_s));

          /* Then perform the send() as sendto() */

          ret = psock_ieee802154_sendto(psock, buf, len, flags,
                                        (FAR const struct sockaddr *)&to,
                                        sizeof(struct sockaddr_ieee802154_s));
        }
    }
  else
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      ret = -EDESTADDRREQ;
    }

  return ret;
}

/****************************************************************************
 * Name: ieee802154_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the PF_IEEE802154
 *   socket.
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

static ssize_t ieee802154_sendto(FAR struct socket *psock, FAR const void *buf,
                                 size_t len, int flags,
                                 FAR const struct sockaddr *to, socklen_t tolen)
{
  ssize_t ret;

  /* Only SOCK_DGRAM is supported (because the MAC header is stripped) */

  if (psock->s_type == SOCK_DGRAM)
    {
      /* Raw packet send */

      ret = psock_ieee802154_sendto(psock, buf, len, flags, to, tolen);
    }
  else
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      ret = -EDESTADDRREQ;
    }

  return ret;
}

/****************************************************************************
 * Name: ieee802154_close
 *
 * Description:
 *   Performs the close operation on a PF_IEEE802154 socket instance
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

static int ieee802154_close(FAR struct socket *psock)
{
  /* Perform some pre-close operations for the PF_IEEE802154 address type */

  switch (psock->s_type)
    {
      case SOCK_DGRAM:
        {
          FAR struct ieee802154_conn_s *conn = psock->s_conn;

          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          if (conn->crefs <= 1)
            {
              /* Yes... free the connection structure */

              conn->crefs = 0;          /* No more references on the connection */
              ieee802154_conn_free(psock->s_conn);  /* Free network resources */
            }
          else
            {
              /* No.. Just decrement the reference count */

              conn->crefs--;
            }

          return OK;
        }

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_IEEE802154 */
