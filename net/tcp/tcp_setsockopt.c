/****************************************************************************
 * net/tcp/tcp_setsockopt.c
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
 * Name: tcp_setsockopt
 *
 * Description:
 *   tcp_setsockopt() sets the TCP-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/tcp.h> for the a complete list of values of TCP protocol
 *   options.
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

int tcp_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len)
{
  FAR struct tcp_conn_s *conn;
  int ret = OK;

  DEBUGASSERT(psock != NULL && value != NULL && psock->s_conn != NULL);
  conn = psock->s_conn;

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
      case SO_KEEPALIVE: /* Verifies TCP connections active by enabling the
                          * periodic transmission of probes */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int keepalive = *(FAR int *)value;

            if (keepalive != 0 && keepalive != 1)
              {
                nerr("ERROR: SO_KEEPALIVE value out of range: %d\n",
                     keepalive);
                return -EDOM;
              }
            else
              {
                conn->keepalive = keepalive;

                /* Reset timer */

                tcp_update_keeptimer(conn, keepalive ? conn->keepidle : 0);
                conn->keepretries = 0;
              }
          }
        break;

      case TCP_NODELAY: /* Avoid coalescing of small segments. */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int nodelay = *(FAR int *)value;

            if (!nodelay)
              {
                nerr("ERROR: TCP_NODELAY not supported\n");
                ret = -ENOSYS;
              }
          }
        break;

      case TCP_KEEPIDLE:  /* Start keepalives after this IDLE period */
      case TCP_KEEPINTVL: /* Interval between keepalives */
        {
          unsigned int dsecs;

          if (value == NULL)
            {
              return -EINVAL;
            }
          else if (value_len == sizeof(struct timeval))
            {
              FAR struct timeval *tv = (FAR struct timeval *)value;

              /* Get the IDLE time value.  Any microsecond remainder will
               * be forced to the next larger, whole decisecond value.
               */

              dsecs = net_timeval2dsec(tv, TV2DS_CEIL);
            }
          else if (value_len == sizeof(int))
            {
              dsecs = *(FAR int *)value;
            }
          else
            {
              return -EDOM;
            }

          if (dsecs > UINT16_MAX)
             {
               nwarn("WARNING: value out of range: %u\n", dsecs);
               return -EDOM;
             }

          if (option == TCP_KEEPIDLE)
            {
              conn->keepidle = dsecs;
            }
          else
            {
              conn->keepintvl = dsecs;
            }

           /* Reset timer */

          if (conn->keepalive)
            {
              tcp_update_keeptimer(conn, conn->keepidle);
              conn->keepretries = 0;
            }
        }
        break;

      case TCP_KEEPCNT: /* Number of keepalives before death */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int keepcnt = *(FAR int *)value;

            if (keepcnt < 0 || keepcnt > UINT8_MAX)
              {
                nerr("ERROR: TCP_KEEPCNT value out of range: %d\n", keepcnt);
                return -EDOM;
              }
            else
              {
                conn->keepcnt = keepcnt;

                /* Reset time */

                if (conn->keepalive)
                  {
                    tcp_update_keeptimer(conn, conn->keepidle);
                    conn->keepretries = 0;
                  }
              }
          }
        break;
#endif /* CONFIG_NET_TCP_KEEPALIVE */

      case TCP_MAXSEG: /* The maximum segment size */
        if (value_len != sizeof(int))
          {
            ret = -EFAULT;
          }
        else
          {
            int mss = *(FAR int *)value;

            if (conn->tcpstateflags != TCP_ALLOCATED)
              {
                /* Set TCP_MAXSEG in the wrong state, direct return success */

                return OK;
              }

            if (mss < TCP_MIN_MSS || mss > UINT16_MAX)
              {
                nerr("ERROR: TCP_MAXSEG value out of range: %d\n", mss);
                return -EINVAL;
              }
            else
              {
                conn->user_mss = mss;
              }
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
