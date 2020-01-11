/****************************************************************************
 * net/tcp/tcp_setsockopt.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <sys/time.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netinet/tcp.h>

#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "tcp/tcp.h"

#ifdef CONFIG_NET_TCPPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_getsockopt
 *
 * Description:
 *   tcp_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET; to retrieve options at the TCP-protocol level, the level
 *   argument is SOL_CP.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netinet/tcp.h for the case of the TCP protocol).
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

int tcp_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len)
{
#ifdef CONFIG_NET_TCP_KEEPALIVE
  /* Keep alive options are the only TCP protocol socket option currently
   * supported.
   */

  FAR struct tcp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && value_len != NULL &&
              psock->s_conn != NULL);
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* All of the TCP protocol options apply only TCP sockets.  The sockets
   * do not have to be connected.. that might occur later with the KeepAlive
   * already configured.
   */

  if (psock->s_type != SOCK_STREAM)
    {
      nerr("ERROR:  Not a TCP socket\n");
      return -ENOTCONN;
    }

  /* Handle the Keep-Alive option */

  switch (option)
    {
      /* Handle the SO_KEEPALIVE socket-level option.
       *
       * NOTE: SO_KEEPALIVE is not really a socket-level option; it is a
       * protocol-level option.  A given TCP connection may service multiple
       * sockets (via dup'ing of the socket).  There is, however, still only
       * one connection to be monitored and that is a global attribute across
       * all of the clones that may use the underlying connection.
       */

      case SO_KEEPALIVE:  /* Verifies TCP connections active by enabling the
                           * periodic transmission of probes */
        if (*value_len < sizeof(int))
          {
            /* REVISIT: POSIX says that we should truncate the value if it
             * is larger than value_len.   That just doesn't make sense
             * to me in this case.
             */

            ret                = -EINVAL;
          }
        else
          {
            FAR int *keepalive = (FAR int *)value;
            *keepalive         = (int)conn->keepalive;
            *value_len         = sizeof(int);
            ret                = OK;
          }
        break;

      case TCP_NODELAY:  /* Avoid coalescing of small segments. */
        nerr("ERROR: TCP_NODELAY not supported\n");
        ret = -ENOSYS;
        break;

      case TCP_KEEPIDLE:  /* Start keepalives after this IDLE period */
        if (*value_len < sizeof(struct timeval))
          {
            /* REVISIT: POSIX says that we should truncate the value if it
             * is larger than value_len.   That just doesn't make sense
             * to me in this case.
             */

            ret = -EINVAL;
          }
        else
          {
            FAR struct timeval *tv = (FAR struct timeval *)value;

            if (tv == NULL)
              {
                ret        = -EINVAL;
              }
            else
              {
                /* Convert the KeepIdle time from deciseconds to struct
                 * timeval.
                 */

                net_dsec2timeval(conn->keepidle, tv);
                *value_len = sizeof(struct timeval);
                ret        = OK;
              }
          }
        break;

      case TCP_KEEPINTVL: /* Interval between keepalives */
        if (*value_len < sizeof(struct timeval))
          {
            /* REVISIT: POSIX says that we should truncate the value if it
             * is larger than value_len.   That just doesn't make sense
             * to me in this case.
             */

            ret = -EINVAL;
          }
        else
          {
            FAR struct timeval *tv = (FAR struct timeval *)value;

            if (tv == NULL)
              {
                ret        = -EINVAL;
              }
            else
              {
                /* Convert the KeepIdle time from deciseconds to struct
                 * timeval.
                 */

                net_dsec2timeval(conn->keepintvl, tv);
                *value_len = sizeof(struct timeval);
                ret        = OK;
              }
          }
        break;

      case TCP_KEEPCNT:   /* Number of keepalives before death */
        if (*value_len < sizeof(int))
          {
            /* REVISIT: POSIX says that we should truncate the value if it
             * is larger than value_len.   That just doesn't make sense
             * to me in this case.
             */

            ret              = -EINVAL;
          }
        else
          {
            FAR int *keepcnt = (FAR int *)value;
            *keepcnt         = (int)conn->keepcnt;
            *value_len       = sizeof(int);
            ret              = OK;
          }
        break;

      default:
        nerr("ERROR: Unrecognized TCP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
#else
  return -ENOPROTOOPT;
#endif /* CONFIG_NET_TCP_KEEPALIVE */
}

#endif /* CONFIG_NET_TCPPROTO_OPTIONS */
