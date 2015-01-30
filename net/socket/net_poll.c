/****************************************************************************
 * net/socket/net_poll.c
 *
 *   Copyright (C) 2008-2009, 2011-2015 Gregory Nutt. All rights reserved.
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

#include <errno.h>

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "local/local.h"
#include "socket/socket.h"

#if defined(CONFIG_NET) && !defined(CONFIG_DISABLE_POLL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Network polling can only be supported if poll support is provided by TCP,
 * UDP, or LOCAL sockets.
 */

#undef HAVE_NET_POLL
#if defined(HAVE_TCP_POLL) || defined(HAVE_UDP_POLL) || defined(HAVE_LOCAL_POLL)
#  define HAVE_NET_POLL 1
#endif

#ifdef HAVE_NET_POLL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_pollsetup
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

static inline int net_pollsetup(FAR struct socket *psock,
                                FAR struct pollfd *fds)
{
#if defined(HAVE_TCP_POLL) || defined(HAVE_LOCAL_POLL)
  if (psock->s_type == SOCK_STREAM)
    {
#ifdef HAVE_LOCAL_POLL
#ifdef HAVE_TCP_POLL
      if (psock->s_domain == PF_LOCAL)
#endif
        {
          return local_pollsetup(psock, fds);
        }
#endif /* HAVE_LOCAL_POLL */

#ifdef HAVE_TCP_POLL
#ifdef HAVE_LOCAL_POLL
      else
#endif
        {
          return tcp_pollsetup(psock, fds);
        }
#endif /* HAVE_TCP_POLL */
    }
#endif /* HAVE_TCP_POLL || HAVE_LOCAL_POLL */

#if defined(HAVE_UDP_POLL) || defined(HAVE_LOCAL_POLL)
  if (psock->s_type != SOCK_STREAM)
    {
#ifdef HAVE_LOCAL_POLL
#ifdef HAVE_UDP_POLL
      if (psock->s_domain == PF_LOCAL)
#endif
        {
          return local_pollsetup(psock, fds);
        }
#endif /* HAVE_LOCAL_POLL */

#ifdef HAVE_UDP_POLL
#ifdef HAVE_LOCAL_POLL
      else
#endif
        {
          return udp_pollsetup(psock, fds);
        }
#endif /* HAVE_UDP_POLL */
    }
#endif /* HAVE_UDP_POLL || HAVE_LOCAL_POLL */

  return -ENOSYS;
}

/****************************************************************************
 * Function: net_pollteardown
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

static inline int net_pollteardown(FAR struct socket *psock,
                                   FAR struct pollfd *fds)
{
#if defined(HAVE_TCP_POLL) || defined(HAVE_LOCAL_POLL)
  if (psock->s_type == SOCK_STREAM)
    {
#ifdef HAVE_LOCAL_POLL
#ifdef HAVE_TCP_POLL
      if (psock->s_domain == PF_LOCAL)
#endif
        {
          return local_pollteardown(psock, fds);
        }
#endif /* HAVE_LOCAL_POLL */

#ifdef HAVE_TCP_POLL
#ifdef HAVE_LOCAL_POLL
      else
#endif
        {
          return tcp_pollteardown(psock, fds);
        }
#endif /* HAVE_TCP_POLL */
    }
#endif /* HAVE_TCP_POLL || HAVE_LOCAL_POLL */

#if defined(HAVE_UDP_POLL) || defined(HAVE_LOCAL_POLL)
  if (psock->s_type != SOCK_STREAM)
    {
#ifdef HAVE_LOCAL_POLL
#ifdef HAVE_UDP_POLL
      if (psock->s_domain == PF_LOCAL)
#endif
        {
          return local_pollteardown(psock, fds);
        }
#endif /* HAVE_LOCAL_POLL */

#ifdef HAVE_UDP_POLL
#ifdef HAVE_LOCAL_POLL
      else
#endif
        {
          return udp_pollteardown(psock, fds);
        }
#endif /* HAVE_UDP_POLL */
    }
#endif /* HAVE_UDP_POLL || HAVE_LOCAL_POLL */

  return -ENOSYS;
}
#endif /* HAVE_NET_POLL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
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

int psock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup)
{
#ifndef HAVE_NET_POLL
  return -ENOSYS;
#else
  int ret;

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the TCP/IP poll() setup */

      ret = net_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the TCP/IP poll() teardown */

      ret = net_pollteardown(psock, fds);
    }

  return ret;
#endif /* HAVE_NET_POLL */
}

/****************************************************************************
 * Function: net_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   fd    - The socket descriptor of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int net_poll(int sockfd, struct pollfd *fds, bool setup)
{
#ifndef HAVE_NET_POLL
  return -ENOSYS;
#else
  FAR struct socket *psock;

  /* Get the underlying socket structure and verify that the sockfd
   * corresponds to valid, allocated socket
   */

  psock = sockfd_socket(sockfd);
  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Then let psock_poll() do the heavy lifting */

  return psock_poll(psock, fds, setup);
#endif /* HAVE_NET_POLL */
}

#endif /* CONFIG_NET && !CONFIG_DISABLE_POLL */
