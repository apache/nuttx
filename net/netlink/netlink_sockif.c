/****************************************************************************
 * net/netlink/netlink_sockif.c
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
#include <string.h>
#include <poll.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
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
static int  netlink_accept(FAR struct socket *psock,
              FAR struct sockaddr *addr, FAR socklen_t *addrlen,
              FAR struct socket *newsock);
static int  netlink_poll(FAR struct socket *psock, FAR struct pollfd *fds,
              bool setup);
static ssize_t netlink_sendmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags);
static ssize_t netlink_recvmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags);
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
  netlink_sendmsg,      /* si_sendmsg */
  netlink_recvmsg,      /* si_recvmsg */
  netlink_close         /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_setup
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

  if (domain == PF_NETLINK &&
      (type == SOCK_RAW || type == SOCK_DGRAM || type == SOCK_CTRL))
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
  /* Permit vfcntl to set socket to non-blocking */

  return SOCKCAP_NONBLOCKING;
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

  nladdr = (FAR struct sockaddr_nl *)addr;
  conn   = (FAR struct netlink_conn_s *)psock->s_conn;

  conn->pid    = nladdr->nl_pid ? nladdr->nl_pid : nxsched_gettid();
  conn->groups = nladdr->nl_groups;

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
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen != NULL && *addrlen >= sizeof(struct sockaddr_nl));

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Return the address information in the address structure */

  nladdr = (FAR struct sockaddr_nl *)addr;
  memset(nladdr, 0, sizeof(struct sockaddr_nl));

  nladdr->nl_family = AF_NETLINK;
  nladdr->nl_pid    = conn->pid;
  nladdr->nl_groups = conn->groups;

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
  FAR struct sockaddr_nl *nladdr;
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen != NULL && *addrlen >= sizeof(struct sockaddr_nl));

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Return the address information in the address structure */

  nladdr = (FAR struct sockaddr_nl *)addr;
  memset(nladdr, 0, sizeof(struct sockaddr_nl));

  nladdr->nl_family = AF_NETLINK;
  nladdr->nl_pid    = conn->dst_pid;
  nladdr->nl_groups = conn->dst_groups;

  *addrlen = sizeof(struct sockaddr_nl);
  return OK;
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
 *   returned.  See listen() for the set of appropriate error values.
 *
 ****************************************************************************/

static int netlink_listen(FAR struct socket *psock, int backlog)
{
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_connect
 *
 * Description:
 *   Perform a netlink connection
 *
 * Input Parameters:
 *   psock   A reference to the structure of the socket to be connected
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
  FAR struct sockaddr_nl *nladdr;
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen >= sizeof(struct sockaddr_nl));

  /* Save the address information in the connection structure */

  nladdr = (FAR struct sockaddr_nl *)addr;
  conn   = (FAR struct netlink_conn_s *)psock->s_conn;

  conn->dst_pid    = nladdr->nl_pid;
  conn->dst_groups = nladdr->nl_groups;

  return OK;
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
 *   not marked as non-blocking, accept blocks the caller until a
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

static int netlink_accept(FAR struct socket *psock,
                          FAR struct sockaddr *addr, FAR socklen_t *addrlen,
                          FAR struct socket *newsock)
{
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: netlink_response_available
 *
 * Description:
 *   Handle a Netlink response available notification.
 *
 * Input Parameters:
 *   Standard work handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void netlink_response_available(FAR void *arg)
{
  FAR struct netlink_conn_s *conn = arg;

  DEBUGASSERT(conn != NULL);

  /* The following should always be true ... but maybe not in some race
   * condition?
   */

  sched_lock();
  net_lock();

  if (conn->fds != NULL)
    {
      /* Wake up the poll() with POLLIN */

      poll_notify(&conn->fds, 1, POLLIN);
    }
  else
    {
      nwarn("WARNING: Missing references in connection.\n");
    }

  /* Allow another poll() */

  conn->fds = NULL;

  net_unlock();
  sched_unlock();
}

/****************************************************************************
 * Name: netlink_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 *     POLLUP:  Will never be reported
 *     POLLERR: Reported in the event of any failure.
 *     POLLOUT: Always reported if requested.
 *     POLLIN:  Reported if requested but only when pending response data is
 *              available
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
  FAR struct netlink_conn_s *conn;
  int ret = OK;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* If POLLOUT is selected, return immediately (maybe) */

      pollevent_t revents = POLLOUT;

      /* If POLLIN is selected and a response is available, return
       * immediately (maybe).
       */

      net_lock();
      if (netlink_check_response(conn))
        {
          revents |= POLLIN;
        }

      /* But return ONLY if POLLIN and/or POLLIN are included in the
       * requested event set.
       */

      poll_notify(&fds, 1, revents);
      if (fds->revents != 0)
        {
          net_unlock();
          return OK;
        }

      /* Set up to be notified when a response is available if POLLIN is
       * requested.
       */

      if ((fds->events & POLLIN) != 0)
        {
          /* Some limitations:  There can be only a single outstanding POLLIN
           * on the Netlink connection.
           */

          if (conn->fds != NULL)
            {
              nerr("ERROR: Multiple polls() on socket not supported.\n");
              net_unlock();
              return -EBUSY;
            }

          /* Set up the notification */

          conn->fds = fds;

          ret = netlink_notifier_setup(netlink_response_available,
                                       conn, conn);
          if (ret < 0)
            {
              nerr("ERROR: netlink_notifier_setup() failed: %d\n", ret);
              conn->fds = NULL;
            }
        }

      net_unlock();
    }
  else
    {
      /* Cancel any response notifications */

      netlink_notifier_teardown(conn);
      conn->fds = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_sendmsg
 *
 * Description:
 *   If sendmsg() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters 'msg_name' and 'msg_namelen' are ignored (and the
 *   error EISCONN may be returned when they are not NULL and 0), and the
 *   error ENOTCONN is returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A reference to the structure of the socket to be connected
 *   msg      msg to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see sendmsg() for the list of appropriate error
 *   values.
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t netlink_sendmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags)
{
  FAR const void *buf = msg->msg_iov->iov_base;
  FAR const struct sockaddr *to = msg->msg_name;
  socklen_t tolen = msg->msg_namelen;
  FAR struct netlink_conn_s *conn;
  FAR struct nlmsghdr *nlmsg;
  struct sockaddr_nl nladdr;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);

  /* Validity check, only single iov supported */

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  /* Get the underlying connection structure */

  conn = (FAR struct netlink_conn_s *)psock->s_conn;
  if (to == NULL)
    {
      /* netlink_send() */

      /* Format the address */

      nladdr.nl_family = AF_NETLINK;
      nladdr.nl_pad    = 0;
      nladdr.nl_pid    = conn->dst_pid;
      nladdr.nl_groups = conn->dst_groups;

      to = (FAR const struct sockaddr *)&nladdr;
      tolen = sizeof(struct sockaddr_nl);
    }

  DEBUGASSERT(tolen >= sizeof(struct sockaddr_nl));

  /* Get a reference to the netlink message */

  nlmsg = (FAR struct nlmsghdr *)buf;
  DEBUGASSERT(nlmsg->nlmsg_len >= sizeof(struct nlmsghdr));

  switch (conn->protocol)
    {
#ifdef CONFIG_NETLINK_ROUTE
      case NETLINK_ROUTE:
        ret = netlink_route_sendto(conn, nlmsg,
                                   msg->msg_iov->iov_len, flags,
                                   (FAR const struct sockaddr_nl *)to,
                                   tolen);
        break;
#endif

      default:
       ret = -EOPNOTSUPP;
       break;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If msg_name is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument 'msg_namelen' is
 *   initialized to the size of the buffer associated with msg_name, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored)
 *
 ****************************************************************************/

static ssize_t netlink_recvmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct netlink_response_s *entry;
  FAR struct socket_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && buf != NULL);
  DEBUGASSERT(from == NULL ||
              (fromlen != NULL && *fromlen >= sizeof(struct sockaddr_nl)));

  /* Find the response to this message.  The return value */

  entry = netlink_tryget_response(psock->s_conn);
  if (entry == NULL)
    {
      conn = psock->s_conn;

      /* No response is variable, but presumably, one is expected.  Check
       * if the socket has been configured for non-blocking operation.
       */

      if (_SS_ISNONBLOCK(conn->s_flags) || (flags & MSG_DONTWAIT) != 0)
        {
          return -EAGAIN;
        }

      /* Wait for the response.  This should always succeed. */

      entry = netlink_get_response(psock->s_conn);
      DEBUGASSERT(entry != NULL);
      if (entry == NULL)
        {
          return -EPIPE;
        }
    }

  if (len > entry->msg.nlmsg_len)
    {
      len = entry->msg.nlmsg_len;
    }

  /* Copy the payload to the user buffer */

  memcpy(buf, &entry->msg, len);
  kmm_free(entry);

  if (from != NULL)
    {
      netlink_getpeername(psock, from, fromlen);
    }

  return len;
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
