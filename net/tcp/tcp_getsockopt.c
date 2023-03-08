/****************************************************************************
 * net/tcp/tcp_getsockopt.c
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
 *   argument is SOL_TCP.
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

#ifdef CONFIG_NET_TCP_KEEPALIVE
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
            *keepalive         = conn->keepalive;
            *value_len         = sizeof(int);
            ret                = OK;
          }
        break;

      case TCP_NODELAY:  /* Avoid coalescing of small segments. */
        if (*value_len < sizeof(int))
          {
            ret                = -EINVAL;
          }
        else
          {
            FAR int *nodelay   = (FAR int *)value;

            /* Always true here since we do not support Nagle. */

            *nodelay           = 1;
            *value_len         = sizeof(int);
            ret                = OK;
          }
        break;

      case TCP_KEEPIDLE:  /* Start keepalives after this IDLE period */
      case TCP_KEEPINTVL: /* Interval between keepalives */
        {
          int dsecs;

          if (option == TCP_KEEPIDLE)
            {
              dsecs = conn->keepidle;
            }
          else
            {
              dsecs = conn->keepintvl;
            }

          if (value == NULL)
            {
              ret = -EINVAL;
            }
          else if (*value_len == sizeof(struct timeval))
            {
              FAR struct timeval *tv = (FAR struct timeval *)value;

              /* Convert the KeepIdle time from deciseconds to struct
               * timeval.
               */

              net_dsec2timeval(dsecs, tv);
              *value_len      = sizeof(struct timeval);
              ret             = OK;
            }
          else if (*value_len == sizeof(int))
            {
              FAR int *pdsecs = (FAR int *)value;
              *pdsecs         = dsecs;
              ret             = OK;
            }
          else
            {
              /* REVISIT: POSIX says that we should truncate the value if it
               * is larger than value_len.   That just doesn't make sense
               * to me in this case.
               */

              ret = -EINVAL;
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
            *keepcnt         = conn->keepcnt;
            *value_len       = sizeof(int);
            ret              = OK;
          }
        break;
#endif /* CONFIG_NET_TCP_KEEPALIVE */

      case TCP_MAXSEG:   /* The maximum segment size */
        if (*value_len < sizeof(int))
          {
            /* REVISIT: POSIX says that we should truncate the value if it
             * is larger than value_len.   That just doesn't make sense
             * to me in this case.
             */

            ret          = -EINVAL;
          }
        else
          {
            FAR int *mss = (FAR int *)value;
            *mss         = conn->mss;
            *value_len   = sizeof(int);
            ret          = OK;
          }
        break;

      default:
        nerr("ERROR: Unrecognized TCP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_TCPPROTO_OPTIONS */
