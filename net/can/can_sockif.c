/****************************************************************************
 * net/can/can_sockif.c
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

#include "can/can.h"
#include "netdev/netdev.h"

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  can_setup(FAR struct socket *psock);
static sockcaps_t can_sockcaps(FAR struct socket *psock);
static void can_addref(FAR struct socket *psock);
static int  can_bind(FAR struct socket *psock,
              FAR const struct sockaddr *addr, socklen_t addrlen);
static int  can_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
              FAR socklen_t *addrlen, FAR struct socket *newsock);
static int  can_poll_local(FAR struct socket *psock, FAR struct pollfd *fds,
              bool setup);
static int can_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_can_sockif =
{
  can_setup,        /* si_setup */
  can_sockcaps,     /* si_sockcaps */
  can_addref,       /* si_addref */
  can_bind,         /* si_bind */
  NULL,             /* si_getsockname */
  NULL,             /* si_getpeername */
  NULL,             /* si_listen */
  NULL,             /* si_connect */
  can_accept,       /* si_accept */
  can_poll_local,   /* si_poll */
  can_sendmsg,      /* si_sendmsg */
  can_recvmsg,      /* si_recvmsg */
  can_close,        /* si_close */
  NULL,             /* si_ioctl */
  NULL,             /* si_socketpair */
  NULL              /* si_shutdown */
#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_CANPROTO_OPTIONS)
  , can_getsockopt  /* si_getsockopt */
  , can_setsockopt  /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual CAN receive operation
 *   via the device interface layer. from can_input()
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct can_poll_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static uint16_t can_poll_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvpriv, uint16_t flags)
{
  FAR struct can_poll_s *info = pvpriv;

  DEBUGASSERT(!info || (info->psock && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & CAN_NEWDATA) != 0)
        {
          eventset |= POLLIN;
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= (POLLHUP | POLLERR);
        }

      /* A poll is a sign that we are free to send data. */

      else if ((flags & CAN_POLL) != 0 &&
                 psock_can_cansend(info->psock) >= 0)
        {
          eventset |= POLLOUT;
        }

      /* Awaken the caller of poll() is requested event occurred. */

      poll_notify(&info->fds, 1, eventset);
    }

  return flags;
}

/****************************************************************************
 * Name: can_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    - A pointer to a user allocated socket structure to be
 *              initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int can_setup(FAR struct socket *psock)
{
  int domain = psock->s_domain;
  int type = psock->s_type;
  int proto = psock->s_proto;

  /* Verify that the protocol is supported */

  DEBUGASSERT((unsigned int)proto <= UINT8_MAX);

  switch (proto)
    {
      case 0:            /* INET subsystem for netlib_ifup */
      case CAN_RAW:      /* RAW sockets */
      case CAN_BCM:      /* Broadcast Manager */
      case CAN_TP16:     /* VAG Transport Protocol v1.6 */
      case CAN_TP20:     /* VAG Transport Protocol v2.0 */
      case CAN_MCNET:    /* Bosch MCNet */
      case CAN_ISOTP:    /* ISO 15765-2 Transport Protocol */
      case CAN_J1939:    /* SAE J1939 */
        break;

      default:
        return -EPROTONOSUPPORT;
    }

  /* Verify the socket type (domain should always be PF_CAN here) */

  if (domain == PF_CAN &&
      (type == SOCK_RAW || type == SOCK_DGRAM || type == SOCK_CTRL))
    {
      /* Allocate the CAN socket connection structure and save it in the
       * new socket instance.
       */

      FAR struct can_conn_s *conn = can_alloc();
      if (conn == NULL)
        {
          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

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
 * Name: can_sockcaps
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

static sockcaps_t can_sockcaps(FAR struct socket *psock)
{
  /* Permit vfcntl to set socket to non-blocking */

  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: can_addref
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

static void can_addref(FAR struct socket *psock)
{
  FAR struct can_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: can_bind
 *
 * Description:
 *   can_bind() gives the socket 'conn' the local address 'addr'. 'addr'
 *   is 'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   conn     CAN socket connection structure
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

static int can_bind(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct sockaddr_can *canaddr;
  FAR struct can_conn_s *conn;
  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && addr != NULL &&
              addrlen >= sizeof(struct sockaddr_can));

  /* Save the address information in the connection structure */

  canaddr = (FAR struct sockaddr_can *)addr;
  conn    = (FAR struct can_conn_s *)psock->s_conn;

  /* Bind CAN device to socket */

#ifdef CONFIG_NETDEV_IFINDEX
  conn->dev = netdev_findbyindex(canaddr->can_ifindex);
#else
  char netdev_name[5] = "can0";
  netdev_name[3] += canaddr->can_ifindex;
  conn->dev = netdev_findbyname((const char *)&netdev_name);
#endif

  return OK;
}

/****************************************************************************
 * Name: can_accept
 *
 * Description:
 *   The can_accept function is used with connection-based socket
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

static int can_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                      FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: can_poll_local
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

static int can_poll_local(FAR struct socket *psock, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct can_conn_s *conn;
  FAR struct can_poll_s *info;
  FAR struct devif_callback_s *cb;
  pollevent_t eventset = 0;
  int ret = OK;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct can_conn_s *)psock->s_conn;
  info = conn->pollinfo;

  /* FIXME add NETDEV_DOWN support */

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      net_lock();

      info->dev = conn->dev;

      cb = can_callback_alloc(info->dev, conn);
      if (cb == NULL)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }

      /* Initialize the poll info container */

      info->psock  = psock;
      info->fds    = fds;
      info->cb     = cb;

      /* Initialize the callback structure.  Save the reference to the info
       * structure as callback private data so that it will be available
       * during callback processing.
       */

      cb->flags    = NETDEV_DOWN;
      cb->priv     = (FAR void *)info;
      cb->event    = can_poll_eventhandler;

      if ((fds->events & POLLOUT) != 0)
        {
          cb->flags |= CAN_POLL;
        }

      if ((fds->events & POLLIN) != 0)
        {
          cb->flags |= CAN_NEWDATA;
        }

      /* Save the reference in the poll info structure as fds private as well
       * for use during poll teardown as well.
       */

      fds->priv = (FAR void *)info;

      /* Check for read data availability now */

      if (!IOB_QEMPTY(&conn->readahead))
        {
          /* Normal data may be read without blocking. */

          eventset |= POLLRDNORM;
        }

      if (psock_can_cansend(psock) >= 0)
        {
          /* A CAN frame may be sent without blocking. */

          eventset |= POLLWRNORM;
        }

      /* Check if any requested events are already in effect */

      poll_notify(&fds, 1, eventset);

errout_with_lock:
      net_unlock();
    }
  else
    {
      info = (FAR struct can_poll_s *)fds->priv;

      if (info != NULL)
        {
          /* Cancel any response notifications */

          can_callback_free(info->dev, conn, info->cb);

          /* Release the poll/select data slot */

          info->fds->priv = NULL;

          /* Then free the poll info container */

          info->psock = NULL;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: can_close
 *
 * Description:
 *   Performs the close operation on a CAN socket instance
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

static int can_close(FAR struct socket *psock)
{
  FAR struct can_conn_s *conn = psock->s_conn;
  int ret = OK;

  /* Perform some pre-close operations for the CAN socket type. */

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform user-space daemon of socket close. */

#warning Missing logic

      /* Free the connection structure */

      conn->crefs = 0;
      can_free(psock->s_conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: can_close failed: %d\n", ret);
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

#endif /* CONFIG_NET_CAN */
