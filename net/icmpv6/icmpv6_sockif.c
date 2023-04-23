/****************************************************************************
 * net/icmpv6/icmpv6_sockif.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <socket/socket.h>

#include "icmpv6/icmpv6.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_ICMPv6_SOCKET

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        icmpv6_setup(FAR struct socket *psock);
static sockcaps_t icmpv6_sockcaps(FAR struct socket *psock);
static void       icmpv6_addref(FAR struct socket *psock);
static int        icmpv6_netpoll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static int        icmpv6_close(FAR struct socket *psock);
#ifdef CONFIG_NET_SOCKOPTS
static int        icmpv6_getsockopt(FAR struct socket *psock, int level,
                    int option, FAR void *value, FAR socklen_t *value_len);
static int        icmpv6_setsockopt(FAR struct socket *psock, int level,
                    int option, FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_icmpv6_sockif =
{
  icmpv6_setup,       /* si_setup */
  icmpv6_sockcaps,    /* si_sockcaps */
  icmpv6_addref,      /* si_addref */
  NULL,               /* si_bind */
  NULL,               /* si_getsockname */
  NULL,               /* si_getpeername */
  NULL,               /* si_listen */
  NULL,               /* si_connect */
  NULL,               /* si_accept */
  icmpv6_netpoll,     /* si_poll */
  icmpv6_sendmsg,     /* si_sendmsg */
  icmpv6_recvmsg,     /* si_recvmsg */
  icmpv6_close,       /* si_close */
  icmpv6_ioctl,       /* si_ioctl */
  NULL,               /* si_socketpair */
  NULL                /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , icmpv6_getsockopt /* si_getsockopt */
  , icmpv6_setsockopt /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int icmpv6_setup(FAR struct socket *psock)
{
  /* SOCK_DGRAM or SOCK_CTRL and IPPROTO_ICMP6 are supported */

  if ((psock->s_type == SOCK_DGRAM || psock->s_type == SOCK_CTRL ||
      psock->s_type == SOCK_RAW) && psock->s_proto == IPPROTO_ICMP6)
    {
      /* Allocate the IPPROTO_ICMP6 socket connection structure and save in
       * the new socket instance.
       */

      FAR struct icmpv6_conn_s *conn = icmpv6_alloc();
      if (conn == NULL)
        {
          /* Failed to reserve a connection structure */

          return -ENOMEM;
        }

      /* Set the reference count on the connection structure.
       * This reference count will be incremented only if the socket is
       * dup'ed
       */

      DEBUGASSERT(conn->crefs == 0);
      conn->crefs = 1;
      if (psock->s_type != SOCK_RAW)
        {
          memset(&conn->filter, 0xff, sizeof(conn->filter));
        }

      /* Save the pre-allocated connection in the socket structure */

      psock->s_conn = conn;
      return OK;
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: icmpv6_sockcaps
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

static sockcaps_t icmpv6_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: icmpv6_addref
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

static void icmpv6_addref(FAR struct socket *psock)
{
  FAR struct icmpv6_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: icmpv6_netpoll
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

static int icmpv6_netpoll(FAR struct socket *psock, FAR struct pollfd *fds,
                        bool setup)
{
  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the ICMPv6 poll() setup */

      return icmpv6_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the ICMPv6 poll() teardown */

      return icmpv6_pollteardown(psock, fds);
    }
}

/****************************************************************************
 * Name: icmpv6_close
 *
 * Description:
 *   Performs the close operation on a raw packet socket instance
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

static int icmpv6_close(FAR struct socket *psock)
{
  FAR struct icmpv6_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = psock->s_conn;

  /* Is this the last reference to the connection structure (there could be\
   * more if the socket was dup'ed).
   */

  DEBUGASSERT(conn->crefs > 0);

  if (conn->crefs <= 1)
    {
      /* Yes... free any read-ahead data */

      iob_free_queue(&conn->readahead);

      /* Then free the connection structure */

      conn->crefs = 0;             /* No more references on the connection */
      icmpv6_free(psock->s_conn);  /* Free network resources */
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return OK;
}

#ifdef CONFIG_NET_SOCKOPTS

/****************************************************************************
 * Name: icmpv6_getsockopt_internal
 *
 * Description:
 *   icmpv6_getsockopt_internal() sets the ICMPV6-protocol socket option
 *   specified by the 'option' argument to the value pointed to by the
 *   'value' argument for the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of ICMPV6 protocol
 *   socket options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.
 *
 ****************************************************************************/

static int icmpv6_getsockopt_internal(FAR struct socket *psock, int option,
                                      FAR void *value,
                                      FAR socklen_t *value_len)
{
  int ret;

  ninfo("option: %d\n", option);

  if (psock->s_type != SOCK_RAW)
    {
      return ENOPROTOOPT;
    }

  net_lock();
  switch (option)
    {
      case ICMP6_FILTER:
        {
          FAR struct icmpv6_conn_s *conn = psock->s_conn;

          if (*value_len > sizeof(struct icmp6_filter))
            {
              *value_len = sizeof(struct icmp6_filter);
            }

          memcpy(value, &conn->filter, *value_len);
          ret = OK;
        }
        break;

      default:
        nerr("ERROR: Unrecognized ICMPV6 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: icmpv6_getsockopt
 *
 * Description:
 *   icmpv6_getsockopt() retrieve the value for the option specified by the
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

static int icmpv6_getsockopt(FAR struct socket *psock, int level, int option,
                             FAR void *value, FAR socklen_t *value_len)
{
  switch (level)
  {
    case IPPROTO_IPV6:
      return ipv6_getsockopt(psock, option, value, value_len);

    case IPPROTO_ICMPV6:
      return icmpv6_getsockopt_internal(psock, option, value, value_len);

    default:
      nerr("ERROR: Unrecognized ICMPV6 option: %d\n", option);
      return -ENOPROTOOPT;
  }
}

/****************************************************************************
 * Name: icmpv6_setsockopt_internal
 *
 * Description:
 *   icmpv6_setsockopt_internal() sets the ICMPV6-protocol socket option
 *   specified by the 'option' argument to the value pointed to by the
 *   'value' argument for the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of ICMPV6 protocol
 *   socket options.
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

static int icmpv6_setsockopt_internal(FAR struct socket *psock, int option,
                                      FAR const void *value,
                                      socklen_t value_len)
{
  int ret;

  ninfo("option: %d\n", option);

  if (psock->s_type != SOCK_RAW)
    {
      return ENOPROTOOPT;
    }

  net_lock();
  switch (option)
    {
      case ICMP6_FILTER:
        {
          FAR struct icmpv6_conn_s *conn = psock->s_conn;

          if (value_len > sizeof(struct icmp6_filter))
            {
              value_len = sizeof(struct icmp6_filter);
            }

          memcpy(&conn->filter, value, value_len);
          ret = OK;
        }
        break;

      default:
        nerr("ERROR: Unrecognized ICMP6 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: icmpv6_setsockopt
 *
 * Description:
 *   icmpv6_setsockopt() sets the option specified by the 'option' argument,
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

static int icmpv6_setsockopt(FAR struct socket *psock, int level, int option,
                             FAR const void *value, socklen_t value_len)
{
  switch (level)
  {
    case IPPROTO_IPV6:
      return ipv6_setsockopt(psock, option, value, value_len);

    case IPPROTO_ICMPV6:
      return icmpv6_setsockopt_internal(psock, option, value, value_len);

    default:
      nerr("ERROR: Unrecognized ICMPV6 option: %d\n", option);
      return -ENOPROTOOPT;
  }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_ICMPv6_SOCKET */
