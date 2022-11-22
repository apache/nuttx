/****************************************************************************
 * net/usrsock/usrsock_sockif.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "usrsock/usrsock.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        usrsock_sockif_setup(FAR struct socket *psock,
                                       int protocol);
static sockcaps_t usrsock_sockif_sockcaps(FAR struct socket *psock);
static void       usrsock_sockif_addref(FAR struct socket *psock);
static int        usrsock_sockif_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_usrsock_sockif =
{
  usrsock_sockif_setup,       /* si_setup */
  usrsock_sockif_sockcaps,    /* si_sockcaps */
  usrsock_sockif_addref,      /* si_addref */
  usrsock_bind,               /* si_bind */
  usrsock_getsockname,        /* si_getsockname */
  usrsock_getpeername,        /* si_getpeername */
  usrsock_listen,             /* si_listen */
  usrsock_connect,            /* si_connect */
  usrsock_accept,             /* si_accept */
  usrsock_poll,               /* si_poll */
  usrsock_sendmsg,            /* si_sendmsg */
  usrsock_recvmsg,            /* si_recvmsg */
  usrsock_sockif_close,       /* si_close */
  usrsock_ioctl,              /* si_ioctl */
  NULL                        /* si_socketpair */
#ifdef CONFIG_NET_SOCKOPTS
  , usrsock_getsockopt        /* si_getsockopt */
  , usrsock_setsockopt        /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_sockif_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Input Parameters:
 *   psock    - A pointer to a user allocated socket structure to be
 *              initialized.
 *   protocol - (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int usrsock_sockif_setup(FAR struct socket *psock, int protocol)
{
  int domain = psock->s_domain;
  int type = psock->s_type;
  int ret;

#ifdef CONFIG_NET_USRSOCK_NO_INET
  if (domain == PF_INET)
    {
      return -ENETDOWN;
    }
#endif

#ifdef CONFIG_NET_USRSOCK_NO_INET6
  if (domain == PF_INET6)
    {
      return -ENETDOWN;
    }
#endif

  if (domain == PF_INET || domain == PF_INET6)
    {
#ifndef CONFIG_NET_USRSOCK_UDP
      if (type == SOCK_DGRAM)
        {
          return -ENETDOWN;
        }
#endif

#ifndef CONFIG_NET_USRSOCK_TCP
      if (type == SOCK_STREAM)
        {
          return -ENETDOWN;
        }
#endif
    }
  else
    {
#ifndef CONFIG_NET_USRSOCK_OTHER
      return -ENETDOWN;
#endif
    }

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

  ret = usrsock_socket(domain, type, protocol, psock);
  if (ret == -ENETDOWN)
    {
      nwarn("WARNING: usrsock daemon is not running\n");
    }

  return ret;
}

/****************************************************************************
 * Name: usrsock_sockif_sockcaps
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

static sockcaps_t usrsock_sockif_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: usrsock_sockif_addref
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

static void usrsock_sockif_addref(FAR struct socket *psock)
{
  FAR struct usrsock_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: usrsock_sockif_close
 *
 * Description:
 *   Performs the close operation on an USRSOCK socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 ****************************************************************************/

static int usrsock_sockif_close(FAR struct socket *psock)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  int ret;

  /* Perform some pre-close operations for the USRSOCK socket type. */

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform user-space daemon of socket close. */

      ret = usrsock_close(conn);

      /* Free the connection structure */

      conn->crefs = 0;
      usrsock_free(psock->s_conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: usrsock_close failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
