/****************************************************************************
 * net/tcp/tcp_setsockopt.c
 *
 *   Copyright (C) 2018, 2020 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET_TCP_KEEPALIVE
  /* Keep alive options are the only TCP protocol socket option currently
   * supported.
   */

  FAR struct tcp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && psock->s_conn != NULL);
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
                conn->keepalive = (bool)keepalive;
                conn->keeptime  = clock_systime_ticks();   /* Reset start time */
                ret = OK;
              }
          }
        break;

      case TCP_NODELAY: /* Avoid coalescing of small segments. */
        nerr("ERROR: TCP_NODELAY not supported\n");
        ret = -ENOSYS;
        break;

      case TCP_KEEPIDLE:  /* Start keepalives after this IDLE period */
        if (value_len != sizeof(struct timeval))
          {
            ret = -EDOM;
          }
        else
          {
            FAR struct timeval *tv = (FAR struct timeval *)value;

            if (tv == NULL)
              {
                ret = -EINVAL;
              }
            else
              {
                unsigned int dsecs;

               /* Get the IDLE time value.  Any microsecond remainder will
                * be forced to the next larger, whole decisecond value.
                */

               dsecs = (socktimeo_t)net_timeval2dsec(tv, TV2DS_CEIL);
               if (dsecs > UINT16_MAX)
                  {
                    nwarn("WARNING: TCP_KEEPIDLE value out of range: %u\n",
                          dsecs);
                    ret = -EDOM;
                  }
                else
                  {
                    conn->keepidle = (uint16_t)dsecs;
                    conn->keeptime = clock_systime_ticks();   /* Reset start time */
                    ret = OK;
                  }
              }
          }
        break;

      case TCP_KEEPINTVL: /* Interval between keepalives */
        if (value_len != sizeof(struct timeval))
          {
            ret = -EDOM;
          }
        else
          {
            FAR struct timeval *tv = (FAR struct timeval *)value;

            if (tv == NULL)
              {
                ret = -EINVAL;
              }
            else
              {
                unsigned int dsecs;

               /* Get the IDLE time value.  Any microsecond remainder will
                * be forced to the next larger, whole decisecond value.
                */

               dsecs = (socktimeo_t)net_timeval2dsec(tv, TV2DS_CEIL);
               if (dsecs > UINT16_MAX)
                  {
                    nwarn("WARNING: TCP_KEEPINTVL value out of range: %u\n",
                          dsecs);
                    ret = -EDOM;
                  }
                else
                  {
                    conn->keepintvl = (uint16_t)dsecs;
                    conn->keeptime  = clock_systime_ticks();   /* Reset start time */
                    ret = OK;
                  }
              }
          }
        break;

      case TCP_KEEPCNT:   /* Number of keepalives before death */
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
                conn->keepcnt  = (uint8_t)keepcnt;
                conn->keeptime = clock_systime_ticks();   /* Reset start time */
                ret = OK;
              }
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
