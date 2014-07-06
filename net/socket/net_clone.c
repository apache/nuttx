/****************************************************************************
 * net/socket/net_clone.c
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/socket.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/net/net.h>
#include <nuttx/net/udp.h>

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_clone
 *
 * Description:
 *   Performs the low level, common portion of net_dup() and net_dup2()
 *
 ****************************************************************************/

int net_clone(FAR struct socket *psock1, FAR struct socket *psock2)
{
  net_lock_t flags;
  int ret = OK;

  /* Parts of this operation need to be atomic */

  flags = net_lock();

  /* Duplicate the socket state */

  psock2->s_type     = psock1->s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  psock2->s_flags    = psock1->s_flags;     /* See _SF_* definitions */
#ifdef CONFIG_NET_SOCKOPTS
  psock2->s_options  = psock1->s_options;   /* Selected socket options */
#ifndef CONFIG_DISABLE_CLOCK
  psock2->s_rcvtimeo = psock1->s_rcvtimeo;  /* Receive timeout value (in deciseconds) */
  psock2->s_sndtimeo = psock1->s_sndtimeo;  /* Send timeout value (in deciseconds) */
#ifdef CONFIG_NET_SOLINGER
  psock2->s_linger   = psock1->s_linger;    /* Linger timeout value (in deciseconds) */
#endif
#endif
#endif
  psock2->s_conn     = psock1->s_conn;      /* UDP or TCP connection structure */
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  psock2->s_sndcb    = NULL;                /* Force allocation of new callback
                                             * instance for TCP send */
#endif

  /* Increment the reference count on the connection */

  DEBUGASSERT(psock2->s_conn);
  psock2->s_crefs    = 1;                   /* One reference on the new socket itself */

#ifdef CONFIG_NET_TCP
  if (psock2->s_type == SOCK_STREAM)
    {
      FAR struct tcp_conn_s *conn = psock2->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
#ifdef CONFIG_NET_UDP
  if (psock2->s_type == SOCK_DGRAM)
    {
      FAR struct udp_conn_s *conn = psock2->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
    {
      ndbg("Unsupported type: %d\n", psock2->s_type);
      ret = -EBADF;
    }

  net_unlock(flags);
  return ret;
}

#endif /* CONFIG_NET */


