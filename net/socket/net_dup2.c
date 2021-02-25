/****************************************************************************
 * net/socket/net_dup2.c
 *
 *   Copyright (C) 2009, 2011-2015, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "inet/inet.h"
#include "tcp/tcp.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_dup2
 *
 * Description:
 *   Performs the low level, common portion of net_dup() and net_dup2()
 *
 * Input Parameters:
 *   psock1 - The existing socket that is being cloned.
 *   psock2 - A reference to an uninitialized socket structure allocated by
 *            the caller.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int psock_dup2(FAR struct socket *psock1, FAR struct socket *psock2)
{
  int ret = OK;

  /* Parts of this operation need to be atomic */

  net_lock();

  /* Duplicate the relevant socket state (zeroing everything else) */

  memset(psock2, 0, sizeof(struct socket));

  psock2->s_domain   = psock1->s_domain;    /* IP domain: PF_INET, PF_INET6, or PF_PACKET */
  psock2->s_type     = psock1->s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  psock2->s_sockif   = psock1->s_sockif;    /* Socket interface */
  psock2->s_flags    = psock1->s_flags;     /* See _SF_* definitions */
#ifdef CONFIG_NET_SOCKOPTS
  psock2->s_options  = psock1->s_options;   /* Selected socket options */
  psock2->s_rcvtimeo = psock1->s_rcvtimeo;  /* Receive timeout value (in deciseconds) */
  psock2->s_sndtimeo = psock1->s_sndtimeo;  /* Send timeout value (in deciseconds) */
#ifdef CONFIG_NET_SOLINGER
  psock2->s_linger   = psock1->s_linger;    /* Linger timeout value (in deciseconds) */
#endif
#endif
  psock2->s_conn     = psock1->s_conn;      /* UDP or TCP connection structure */

  /* Increment the reference count on the socket */

  psock2->s_crefs    = 1;                   /* One reference on the new socket itself */

  /* Increment the reference count on the underlying connection structure
   * for this address family type.
   */

  DEBUGASSERT(psock2->s_sockif != NULL &&
              psock2->s_sockif->si_addref != NULL);
  psock2->s_sockif->si_addref(psock2);

#ifdef NET_TCP_HAVE_STACK
  /* For connected socket types, it is necessary to also start the network
   * monitor so that the newly cloned socket can receive a notification if
   * the network connection is lost.
   */

  if (psock2->s_type == SOCK_STREAM)
    {
      ret = tcp_start_monitor(psock2);

      /* On failure, back out the reference count on the TCP connection
       * structure.  tcp_start_monitor() will fail only in the race condition
       * where the TCP connection has been lost.
       */

      if (ret < 0)
        {
          /* There should be at least two reference counts on the connection
           * structure:  At least one from the original socket and the one
           * from above where we incremented the reference count.
           * inet_close() will handle all cases.
           *
           * NOTE:  As a side-effect, inet_close()will also call
           * tcp_stop_monitor() which could inform the loss of connection to
           * all open sockets on the connection structure if the reference
           * count decrements to zero.
           */

          inet_close(psock2);

          /* Then release our reference on the socket structure containing
           * the connection.
           */

          psock_release(psock2);
        }
    }
#endif

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: net_dup2
 *
 * Description:
 *   Clone a socket descriptor to an arbitrary descriptor number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int net_dup2(int sockfd1, int sockfd2)
{
  FAR struct socket *psock1;
  FAR struct socket *psock2;
  int ret;

  /* Lock the scheduler throughout the following */

  sched_lock();

  /* Get the socket structures underly both descriptors */

  psock1 = sockfd_socket(sockfd1);
  psock2 = sockfd_socket(sockfd2);

  /* Verify that the sockfd1 and sockfd2 both refer to valid socket
   * descriptors and that sockfd2 corresponds to an allocated socket
   */

  if (psock1 == NULL || psock2 == NULL || psock1->s_crefs <= 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* If sockfd2 also valid, allocated socket, then we will have to
   * close it!
   */

  if (psock2->s_crefs > 0)
    {
      net_close(sockfd2);
    }

  /* Duplicate the socket state */

  ret = psock_dup2(psock1, psock2);

errout:
  sched_unlock();
  return ret;
}
