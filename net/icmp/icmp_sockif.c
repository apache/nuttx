/****************************************************************************
 * net/icmp/icmp_sockif.c
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
#include <nuttx/net/icmp.h>
#include <socket/socket.h>

#include "icmp/icmp.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        icmp_setup(FAR struct socket *psock);
static sockcaps_t icmp_sockcaps(FAR struct socket *psock);
static void       icmp_addref(FAR struct socket *psock);
static int        icmp_netpoll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static int        icmp_close(FAR struct socket *psock);
#ifdef CONFIG_NET_SOCKOPTS
static int        icmp_getsockopt(FAR struct socket *psock, int level,
                    int option, FAR void *value, FAR socklen_t *value_len);
static int        icmp_setsockopt(FAR struct socket *psock, int level,
                    int option, FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_icmp_sockif =
{
  icmp_setup,       /* si_setup */
  icmp_sockcaps,    /* si_sockcaps */
  icmp_addref,      /* si_addref */
  NULL,             /* si_bind */
  NULL,             /* si_getsockname */
  NULL,             /* si_getpeername */
  NULL,             /* si_listen */
  NULL,             /* si_connect */
  NULL,             /* si_accept */
  icmp_netpoll,     /* si_poll */
  icmp_sendmsg,     /* si_sendmsg */
  icmp_recvmsg,     /* si_recvmsg */
  icmp_close,       /* si_close */
  icmp_ioctl,       /* si_ioctl */
  NULL,             /* si_socketpair */
  NULL              /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , icmp_getsockopt /* si_getsockopt */
  , icmp_setsockopt /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_setup
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

static int icmp_setup(FAR struct socket *psock)
{
  /* SOCK_DGRAM or SOCK_CTRL and IPPROTO_ICMP are supported */

  if ((psock->s_type == SOCK_DGRAM || psock->s_type == SOCK_CTRL ||
      psock->s_type == SOCK_RAW) && psock->s_proto == IPPROTO_ICMP)
    {
      /* Allocate the IPPROTO_ICMP socket connection structure and save in
       * the new socket instance.
       */

      FAR struct icmp_conn_s *conn = icmp_alloc();
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
          conn->filter = UINT32_MAX;
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
 * Name: icmp_sockcaps
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

static sockcaps_t icmp_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: icmp_addref
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

static void icmp_addref(FAR struct socket *psock)
{
  FAR struct icmp_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: icmp_netpoll
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

static int icmp_netpoll(FAR struct socket *psock, FAR struct pollfd *fds,
                        bool setup)
{
  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the ICMP poll() setup */

      return icmp_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the ICMP poll() teardown */

      return icmp_pollteardown(psock, fds);
    }
}

/****************************************************************************
 * Name: icmp_close
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

static int icmp_close(FAR struct socket *psock)
{
  FAR struct icmp_conn_s *conn;

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

      conn->crefs = 0;           /* No more references on the connection */
      icmp_free(psock->s_conn);  /* Free network resources */
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
 * Name: icmp_getsockopt_internal
 *
 * Description:
 *   icmp_getsockopt_internal() sets the ICMP-protocol socket option
 *   specified by the 'option' argument to the value pointed to by the
 *   'value' argument for the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of ICMP protocol
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

static int icmp_getsockopt_internal(FAR struct socket *psock, int option,
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
      case ICMP_FILTER:
        {
          FAR struct icmp_conn_s *conn = psock->s_conn;

          if (*value_len > sizeof(uint32_t))
            {
              *value_len = sizeof(uint32_t);
            }

          memcpy(value, &conn->filter, *value_len);
          ret = OK;
        }
        break;

      default:
        nerr("ERROR: Unrecognized ICMP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: icmp_getsockopt
 *
 * Description:
 *   icmp_getsockopt() retrieve the value for the option specified by the
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

static int icmp_getsockopt(FAR struct socket *psock, int level, int option,
                           FAR void *value, FAR socklen_t *value_len)
{
  switch (level)
  {
    case IPPROTO_IP:
      return ipv4_getsockopt(psock, option, value, value_len);

    case IPPROTO_ICMP:
      return icmp_getsockopt_internal(psock, option, value, value_len);

    default:
      nerr("ERROR: Unrecognized ICMP option: %d\n", option);
      return -ENOPROTOOPT;
  }
}

/****************************************************************************
 * Name: icmp_setsockopt_internal
 *
 * Description:
 *   icmp_setsockopt_internal() sets the ICMP-protocol socket option
 *   specified by the 'option' argument to the value pointed to by the
 *   'value' argument for the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of ICMP protocol
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

static int icmp_setsockopt_internal(FAR struct socket *psock, int option,
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
      case ICMP_FILTER:
        {
          FAR struct icmp_conn_s *conn = psock->s_conn;

          if (value_len > sizeof(uint32_t))
            {
              value_len = sizeof(uint32_t);
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
 * Name: icmp_setsockopt
 *
 * Description:
 *   icmp_setsockopt() sets the option specified by the 'option' argument,
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

static int icmp_setsockopt(FAR struct socket *psock, int level, int option,
                           FAR const void *value, socklen_t value_len)
{
  switch (level)
  {
    case IPPROTO_IP:
      return ipv4_setsockopt(psock, option, value, value_len);

    case IPPROTO_ICMP:
      return icmp_setsockopt_internal(psock, option, value, value_len);

    default:
      nerr("ERROR: Unrecognized ICMP option: %d\n", option);
      return -ENOPROTOOPT;
  }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_ICMP_SOCKET */
