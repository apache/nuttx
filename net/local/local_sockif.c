/****************************************************************************
 * net/local/local_sockif.c
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
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/net/net.h>
#include <socket/socket.h>

#include "local/local.h"

#ifdef CONFIG_NET_LOCAL

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        local_setup(FAR struct socket *psock);
static sockcaps_t local_sockcaps(FAR struct socket *psock);
static void       local_sockaddref(FAR struct socket *psock);
static int        local_bind(FAR struct socket *psock,
                             FAR const struct sockaddr *addr,
                             socklen_t addrlen);
static int        local_getsockname(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen);
static int        local_getpeername(FAR struct socket *psock,
                                    FAR struct sockaddr *addr,
                                    FAR socklen_t *addrlen);
static int        local_connect(FAR struct socket *psock,
                                FAR const struct sockaddr *addr,
                                socklen_t addrlen);
static int        local_poll(FAR struct socket *psock,
                             FAR struct pollfd *fds, bool setup);
static int        local_close(FAR struct socket *psock);
static int        local_ioctl(FAR struct socket *psock,
                              int cmd, unsigned long arg);
static int        local_socketpair(FAR struct socket *psocks[2]);
static int        local_shutdown(FAR struct socket *psock, int how);
#ifdef CONFIG_NET_SOCKOPTS
static int        local_getsockopt(FAR struct socket *psock, int level,
                                   int option, FAR void *value,
                                   FAR socklen_t *value_len);
static int        local_setsockopt(FAR struct socket *psock, int level,
                                   int option, FAR const void *value,
                                   socklen_t value_len);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_local_sockif =
{
  local_setup,       /* si_setup */
  local_sockcaps,    /* si_sockcaps */
  local_sockaddref,  /* si_addref */
  local_bind,        /* si_bind */
  local_getsockname, /* si_getsockname */
  local_getpeername, /* si_getpeername */
#ifdef CONFIG_NET_LOCAL_STREAM
  local_listen,      /* si_listen */
#else
  NULL,              /* si_listen */
#endif
  local_connect,     /* si_connect */
#ifdef CONFIG_NET_LOCAL_STREAM
  local_accept,      /* si_accept */
#else
  NULL,              /* si_accept */
#endif
  local_poll,        /* si_poll */
  local_sendmsg,     /* si_sendmsg */
  local_recvmsg,     /* si_recvmsg */
  local_close,       /* si_close */
  local_ioctl,       /* si_ioctl */
  local_socketpair,  /* si_socketpair */
  local_shutdown     /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , local_getsockopt /* si_getsockopt */
  , local_setsockopt /* si_setsockopt */
#endif
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

  local_addref(conn);

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
#if defined(CONFIG_NET_LOCAL_STREAM)
  conn->lc_psock = psock;
#endif
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
 *   psock    A pointer to a user allocated socket structure
 *            to be initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int local_setup(FAR struct socket *psock)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * connection structure, it is unallocated at this point.  It will not
   * actually be initialized until the socket is connected.
   *
   * REVISIT:  Only SOCK_STREAM and SOCK_DGRAM are supported.  Should also
   * support SOCK_RAW.
   */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
        if (psock->s_proto != 0 && psock->s_proto != IPPROTO_TCP)
          {
            return -EPROTONOSUPPORT;
          }

        /* Allocate and attach the local connection structure */

        return local_sockif_alloc(psock);
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        if (psock->s_proto != 0 && psock->s_proto != IPPROTO_UDP)
          {
            return -EPROTONOSUPPORT;
          }

        /* Allocate and attach the local connection structure */

        return local_sockif_alloc(psock);
#endif /* CONFIG_NET_LOCAL_DGRAM */

#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
      case SOCK_CTRL:
        if (psock->s_proto == 0 || psock->s_proto == IPPROTO_TCP ||
            psock->s_proto == IPPROTO_UDP)
          {
            /* Allocate and attach the local connection structure */

            return local_sockif_alloc(psock);
          }

        return -EPROTONOSUPPORT;
#endif

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
 * Name: local_sockaddref
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

static void local_sockaddref(FAR struct socket *psock)
{
  DEBUGASSERT(psock->s_domain == PF_LOCAL);
  local_addref(psock->s_conn);
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
      nerr("ERROR: Invalid address length: %d < %zu\n",
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
      case SOCK_CTRL:
        {
          /* Bind the Unix domain connection structure */

          ret = psock_local_bind(psock, addr, addrlen);
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

  if (*addrlen < sizeof(sa_family_t))
    {
      /* This is apparently not an error */

      *addrlen = 0;
      return OK;
    }

  conn = psock->s_conn;

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
      else /* conn->lc_type = LOCAL_TYPE_PATHNAME */
        {
          /* Get the full length of the socket name (incl. null terminator) */

          size_t namelen = strlen(conn->lc_path) + 1 +
                           (conn->lc_type == LOCAL_TYPE_ABSTRACT);

          /* Get the available length in the user-provided buffer. */

          size_t pathlen = *addrlen - sizeof(sa_family_t);

          /* Clip the socket name size so that if fits in the user buffer */

          if (pathlen < namelen)
            {
              namelen = pathlen;
            }

          /* Copy the path into the user address structure */

          if (conn->lc_type == LOCAL_TYPE_ABSTRACT)
            {
              unaddr->sun_path[0] = '\0';
              strlcpy(&unaddr->sun_path[1], conn->lc_path, namelen - 1);
            }
          else
            {
              strlcpy(unaddr->sun_path, conn->lc_path, namelen);
            }

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
  FAR struct sockaddr_un *unaddr = (FAR struct sockaddr_un *)addr;
  FAR struct local_conn_s *conn;
  FAR struct local_conn_s *peer;

  if (*addrlen < sizeof(sa_family_t))
    {
      /* This is apparently not an error */

      *addrlen = 0;
      return OK;
    }

  /* Verify that the socket has been connected */

  conn = psock->s_conn;

  if (conn->lc_state != LOCAL_STATE_CONNECTED)
    {
      return -ENOTCONN;
    }

  peer = conn->lc_peer;

  /* Save the address family */

  unaddr->sun_family = AF_LOCAL;
  if (*addrlen > sizeof(sa_family_t))
    {
      /* Now copy the address description.  */

      if (peer->lc_type == LOCAL_TYPE_UNNAMED)
        {
          /* Zero-length sun_path... This is an abstract Unix domain socket */

          *addrlen = sizeof(sa_family_t);
        }
      else /* conn->lc_type = LOCAL_TYPE_PATHNAME */
        {
          /* Get the full length of the socket name (incl. null terminator) */

          size_t namelen = strlen(peer->lc_path) + 1 +
                           (peer->lc_type == LOCAL_TYPE_ABSTRACT);

          /* Get the available length in the user-provided buffer. */

          size_t pathlen = *addrlen - sizeof(sa_family_t);

          /* Clip the socket name size so that if fits in the user buffer */

          if (pathlen < namelen)
            {
              namelen = pathlen;
            }

          /* Copy the path into the user address structure */

          if (peer->lc_type == LOCAL_TYPE_ABSTRACT)
            {
              unaddr->sun_path[0] = '\0';
              strlcpy(&unaddr->sun_path[1],
                      peer->lc_path, namelen - 1);
            }
          else
            {
               strlcpy(unaddr->sun_path,
                      peer->lc_path, namelen);
            }

          *addrlen = sizeof(sa_family_t) + namelen;
        }
    }

  return OK;
}

#ifdef CONFIG_NET_SOCKOPTS

/****************************************************************************
 * Name: local_getsockopt
 *
 * Description:
 *   local_getsockopt() retrieve the value for the option specified by the
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

static int local_getsockopt(FAR struct socket *psock, int level, int option,
                            FAR void *value, FAR socklen_t *value_len)
{
  DEBUGASSERT(psock->s_domain == PF_LOCAL);

  if (level == SOL_SOCKET)
    {
      switch (option)
        {
#ifdef CONFIG_NET_LOCAL_SCM
          case SO_PEERCRED:
            {
              FAR struct local_conn_s *conn = psock->s_conn;
              if (*value_len != sizeof(struct ucred))
                {
                  return -EINVAL;
                }

              memcpy(value, &conn->lc_peer->lc_cred, sizeof(struct ucred));
              return OK;
            }
#endif

          case SO_SNDBUF:
            {
              if (*value_len != sizeof(int))
                {
                  return -EINVAL;
                }

              *(FAR int *)value = LOCAL_SEND_LIMIT;
              return OK;
            }
        }
    }

  return -ENOPROTOOPT;
}

/****************************************************************************
 * Name: local_setsockopt
 *
 * Description:
 *   local_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the usrsock connection.
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

static int local_setsockopt(FAR struct socket *psock, int level, int option,
                            FAR const void *value, socklen_t value_len)
{
  return -ENOPROTOOPT;
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
 *   0 on success; a negated errno value on failure.  See connect() for the
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
          FAR struct socket_conn_s *conn = psock->s_conn;

          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(conn->s_flags))
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

          /* #warning Missing logic */

          return -ENOSYS;
        }
        break;
#endif /* CONFIG_NET_LOCAL_DGRAM */

#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
      case SOCK_CTRL:
        {
          return -ENOSYS;
        }
        break;
#endif

      default:
        return -EBADF;
    }
}

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

static int local_poll(FAR struct socket *psock, FAR struct pollfd *fds,
                      bool setup)
{
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
      case SOCK_CTRL:
        {
          /* Is this the last reference to the connection structure (there
           * could be more if the socket was dup'ed).
           */

          local_subref(psock->s_conn);

          return OK;
        }
#endif /* CONFIG_NET_LOCAL_STREAM || CONFIG_NET_LOCAL_DGRAM */

      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Name: local_ioctl
 *
 * Description:
 *   This function performs local device specific operations.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

static int local_ioctl(FAR struct socket *psock, int cmd, unsigned long arg)
{
  FAR struct local_conn_s *conn;
  int ret = OK;

  conn = psock->s_conn;

  switch (cmd)
    {
      case FIONBIO:
        if (conn->lc_infile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_infile, cmd, arg);
          }

        if (ret >= 0 && conn->lc_outfile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_outfile, cmd, arg);
          }
        break;
      case FIONREAD:
        if (conn->lc_infile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_infile, cmd, arg);
          }
        else
          {
            ret = -ENOTCONN;
          }
        break;
      case FIONWRITE:
      case FIONSPACE:
        if (conn->lc_outfile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_outfile, cmd, arg);
          }
        else
          {
            ret = -ENOTCONN;
          }
        break;
      case PIPEIOC_POLLINTHRD:
        if (conn->lc_infile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_infile, cmd, arg);
          }
        else
          {
            ret = -ENOTCONN;
          }
        break;
      case PIPEIOC_POLLOUTTHRD:
        if (conn->lc_outfile.f_inode != NULL)
          {
            ret = file_ioctl(&conn->lc_outfile, cmd, arg);
          }
        else
          {
            ret = -ENOTCONN;
          }
        break;
      case FIOC_FILEPATH:
        snprintf((FAR char *)(uintptr_t)arg, PATH_MAX, "local:[%s]",
                 conn->lc_path);
        break;
      case BIOC_FLUSH:
        ret = -EINVAL;
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: local_socketpair
 *
 * Description:
 *   Create a pair of connected sockets between psocks[2]
 *
 * Parameters:
 *   psocks  A reference to the socket structure of the socket pair
 *
 ****************************************************************************/

static int local_socketpair(FAR struct socket *psocks[2])
{
  FAR struct local_conn_s *conns[2];
  bool nonblock;
  int ret;
  int i;

  for (i = 0; i < 2; i++)
    {
      conns[i] = psocks[i]->s_conn;
      snprintf(conns[i]->lc_path,
               sizeof(conns[i]->lc_path), "socketpair%p", psocks[0]);

      conns[i]->lc_proto = psocks[i]->s_type;
      conns[i]->lc_type  = LOCAL_TYPE_PATHNAME;
      conns[i]->lc_state = LOCAL_STATE_BOUND;
    }

  conns[0]->lc_instance_id = conns[1]->lc_instance_id
#ifdef CONFIG_NET_LOCAL_STREAM
                           = local_generate_instance_id();
#else
                           = -1;
#endif

  /* Create the FIFOs needed for the connection */

  ret = local_create_fifos(conns[0]);
  if (ret < 0)
    {
      goto errout;
    }

  nonblock = _SS_ISNONBLOCK(conns[0]->lc_conn.s_flags);

  /* Open the client-side write-only FIFO. */

  ret = local_open_client_tx(conns[0], nonblock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Open the server-side read-only FIFO. */

  ret = local_open_server_rx(conns[1], nonblock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Open the server-side write-only FIFO. */

  ret = local_open_server_tx(conns[1], nonblock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Open the client-side read-only FIFO */

  ret = local_open_client_rx(conns[0], nonblock);
  if (ret < 0)
    {
      goto errout;
    }

  conns[0]->lc_state = conns[1]->lc_state
                     = LOCAL_STATE_CONNECTED;

#ifdef CONFIG_NET_LOCAL_DGRAM
  if (psocks[0]->s_type == SOCK_DGRAM)
    {
      for (i = 0; i < 2; i++)
        {
          ret = local_set_pollthreshold(conns[i], sizeof(uint16_t));
        }
    }
#endif

  return OK;

errout:
  local_release_fifos(conns[0]);
  return ret;
}

/****************************************************************************
 * Name: local_shutdown
 *
 * Description:
 *   The shutdown() function will cause all or part of a full-duplex
 *   connection on the socket associated with the file descriptor socket to
 *   be shut down.
 *
 *   The shutdown() function disables subsequent send and/or receive
 *   operations on a socket, depending on the value of the how argument.
 *
 * Input Parameters:
 *   sockfd - Specifies the file descriptor of the socket.
 *   how    - Specifies the type of shutdown. The values are as follows:
 *
 *     SHUT_RD   - Disables further receive operations.
 *     SHUT_WR   - Disables further send operations.
 *     SHUT_RDWR - Disables further send and receive operations.
 *
 ****************************************************************************/

static int local_shutdown(FAR struct socket *psock, int how)
{
  DEBUGASSERT(psock->s_domain == PF_LOCAL);

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_LOCAL_STREAM
      case SOCK_STREAM:
        {
          FAR struct local_conn_s *conn = psock->s_conn;
          if (how & SHUT_RD)
            {
              if (conn->lc_infile.f_inode != NULL)
                {
                  file_close(&conn->lc_infile);
                  conn->lc_infile.f_inode = NULL;
                }
            }

          if (how & SHUT_WR)
            {
              if (conn->lc_outfile.f_inode != NULL)
                {
                  file_close(&conn->lc_outfile);
                  conn->lc_outfile.f_inode = NULL;
                }
            }
        }

        return OK;
#endif
#ifdef CONFIG_NET_LOCAL_DGRAM
      case SOCK_DGRAM:
        return -EOPNOTSUPP;
#endif
#if defined(CONFIG_NET_LOCAL_STREAM) || defined(CONFIG_NET_LOCAL_DGRAM)
      case SOCK_CTRL:
        return -EOPNOTSUPP;
#endif
      default:
        return -EBADF;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_LOCAL */
