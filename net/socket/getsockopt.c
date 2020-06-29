/****************************************************************************
 * net/socket/getsockopt.c
 *
 *   Copyright (C) 2007-2009, 2012, 2014, 2017-2019 Gregory Nutt. All rights
 *     reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include "socket/socket.h"
#include "tcp/tcp.h"
#include "usrsock/usrsock.h"
#include "utils/utils.h"
#include "can/can.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_socketlevel_option
 *
 * Description:
 *   getsockopt() retrieve the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
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

static int psock_socketlevel_option(FAR struct socket *psock, int option,
                                    FAR void *value,
                                    FAR socklen_t *value_len)
{
  /* Verify that the socket option if valid (but might not be supported ) */

  if (!_SO_GETVALID(option) || !value || !value_len)
    {
      return -EINVAL;
    }

#ifdef CONFIG_NET_USRSOCK
  if (psock->s_type == SOCK_USRSOCK_TYPE)
    {
      FAR struct usrsock_conn_s *conn = psock->s_conn;

      DEBUGASSERT(conn);

      /* Some of the socket options are handled from this function. */

      switch (option)
        {
          case SO_TYPE:     /* Type can be read from NuttX psock structure. */
          case SO_RCVTIMEO: /* Rx timeouts can be handled at NuttX side, thus
                             * simplify daemon implementation. */
          case SO_SNDTIMEO: /* Rx timeouts can be handled at NuttX side, thus
                             * simplify daemon implementation. */
            break;

          default:          /* Other options are passed to usrsock daemon. */
            {
              return usrsock_getsockopt(conn, SOL_SOCKET,
                                        option, value, value_len);
            }
        }
    }
#endif

  /* Process the option */

  switch (option)
    {
      case SO_ACCEPTCONN: /* Reports whether socket listening is enabled */
        if (*value_len < sizeof(int))
          {
            return -EINVAL;
          }

        *(FAR int *)value = _SS_ISLISTENING(psock->s_flags);
        *value_len        = sizeof(int);
        break;

      /* The following options take a point to an integer boolean value.
       * We will blindly report the bit here although the implementation
       * is outside of the scope of getsockopt.
       */

      case SO_BROADCAST:  /* Permits sending of broadcast messages */
      case SO_DEBUG:      /* Enables recording of debugging information */
      case SO_DONTROUTE:  /* Requests outgoing messages bypass standard routing */
#ifndef CONFIG_NET_TCPPROTO_OPTIONS
      case SO_KEEPALIVE:  /* Verifies TCP connections active by enabling the
                           * periodic transmission of probes */
#endif
      case SO_OOBINLINE:  /* Leaves received out-of-band data inline */
      case SO_REUSEADDR:  /* Allow reuse of local addresses */
        {
          sockopt_t optionset;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(int))
            {
              return -EINVAL;
           }

          /* Sample the current options.  This is atomic operation and so
           * should not require any special steps for thread safety. We
           * this outside of the macro because you can never be sure what
           * a macro will do.
           */

          optionset         = psock->s_options;
          *(FAR int *)value = _SO_GETOPT(optionset, option);
          *value_len        = sizeof(int);
        }
        break;

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      /* Any connection-oriented protocol could potentially support
       * SO_KEEPALIVE.  However, this option is currently only available for
       * TCP/IP.
       *
       * NOTE: SO_KEEPALIVE is not really a socket-level option; it is a
       * protocol-level option.  A given TCP connection may service multiple
       * sockets (via dup'ing of the socket).  There is, however, still only
       * one connection to be monitored and that is a global attribute across
       * all of the clones that may use the underlying connection.
       */

      case SO_KEEPALIVE:  /* Verifies TCP connections active by enabling the
                           * periodic transmission of probes */
        return tcp_getsockopt(psock, option, value, value_len);
#endif

      case SO_TYPE:       /* Reports the socket type */
        {
          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(int))
            {
              return -EINVAL;
            }

#ifdef CONFIG_NET_USRSOCK
          if (psock->s_type == SOCK_USRSOCK_TYPE)
            {
              FAR struct usrsock_conn_s *conn = psock->s_conn;

              /* Return the actual socket type */

              *(FAR int *)value = conn->type;
              *value_len        = sizeof(int);

              break;
            }
#endif

          /* Return the socket type */

          *(FAR int *)value = psock->s_type;
          *value_len        = sizeof(int);
        }
        break;

      /* The following are valid only if the OS CLOCK feature is enabled */

      case SO_RCVTIMEO:
      case SO_SNDTIMEO:
        {
          socktimeo_t timeo;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(struct timeval))
            {
              return -EINVAL;
            }

          /* Get the timeout value.  This is a atomic operation and should
         * require no special operation.
         */

          if (option == SO_RCVTIMEO)
            {
              timeo = psock->s_rcvtimeo;
            }
          else
            {
              timeo = psock->s_sndtimeo;
            }

          /* Then return the timeout value to the caller */

          net_dsec2timeval(timeo, (struct timeval *)value);
          *value_len   = sizeof(struct timeval);
        }
        break;

      case SO_ERROR:      /* Reports and clears error status. */
        {
          if (*value_len != sizeof(int))
            {
              return -EINVAL;
            }

          *(FAR int *)value = (int)psock->s_error;
          psock->s_error = 0;
        }
        break;

#ifdef CONFIG_NET_TIMESTAMP
      case SO_TIMESTAMP:
        {
          if (*value_len != sizeof(int))
            {
              return -EINVAL;
            }

          *(FAR int *)value = (int)psock->s_timestamp;
        }
        break;
#endif

      /* The following are not yet implemented
       * (return values other than {0,1})
       */

      case SO_LINGER:     /* Lingers on a close() if data is present */
      case SO_RCVBUF:     /* Sets receive buffer size */
      case SO_RCVLOWAT:   /* Sets the minimum number of bytes to input */
      case SO_SNDBUF:     /* Sets send buffer size */
      case SO_SNDLOWAT:   /* Sets the minimum number of bytes to output */

      default:
        return -ENOPROTOOPT;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve the value for the option specified by the
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
 *   value to indicate the nature of the error.
 *
 *   EINVAL
 *     The specified option is invalid at the specified socket 'level' or the
 *     socket has been shutdown.
 *   ENOPROTOOPT
 *     The 'option' is not supported by the protocol.
 *   ENOTSOCK
 *     The 'psock' argument does not refer to a socket.
 *   ENOBUFS
 *     Insufficient resources are available in the system to complete the
 *     call.
 *
 ****************************************************************************/

int psock_getsockopt(FAR struct socket *psock, int level, int option,
                     FAR void *value, FAR socklen_t *value_len)
{
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Handle retrieval of the socket option according to the level at which
   * option should be applied.
   */

  switch (level)
    {
      case SOL_SOCKET:   /* Socket-level options (see include/sys/socket.h) */
       ret = psock_socketlevel_option(psock, option, value, value_len);
       break;

      case IPPROTO_TCP:  /* TCP protocol socket options (see include/netinet/tcp.h) */
#ifdef CONFIG_NET_TCPPROTO_OPTIONS
       ret = tcp_getsockopt(psock, option, value, value_len);
       break;
#endif

      case SOL_CAN_RAW:/* CAN protocol socket options (see include/netpacket/can.h) */
#ifdef CONFIG_NET_CANPROTO_OPTIONS
       ret = can_getsockopt(psock, option, value, value_len);
#endif
       break;

      /* These levels are defined in sys/socket.h, but are not yet
       * implemented.
       */

      case IPPROTO_IP:   /* TCP protocol socket options (see include/netinet/ip.h) */
      case IPPROTO_IPV6: /* TCP protocol socket options (see include/netinet/ip6.h) */
      case IPPROTO_UDP:  /* TCP protocol socket options (see include/netinit/udp.h) */
        ret = -ENOSYS;
       break;

      default:           /* The provided level is invalid */
        ret = -EINVAL;
       break;
    }

  return ret;
}

/****************************************************************************
 * Name: getsockopt
 *
 * Description:
 *   getsockopt() retrieve the value for the option specified by the
 *   'option' argument for the socket specified by the 'sockfd' argument. If
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
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, -1 (ERROR) is returned and th
 *   errno variable is set appropriately:
 *
 *   EBADF
 *     The 'sockfd' argument is not a valid socket descriptor.
 *   EINVAL
 *     The specified option is invalid at the specified socket 'level' or the
 *     socket has been shutdown.
 *   ENOPROTOOPT
 *     The 'option' is not supported by the protocol.
 *   ENOTSOCK
 *     The 'sockfd' argument does not refer to a socket.
 *   ENOBUFS
 *     Insufficient resources are available in the system to complete the
 *     call.
 *
 ****************************************************************************/

int getsockopt(int sockfd, int level, int option,
               void *value, socklen_t *value_len)
{
  FAR struct socket *psock;
  int ret;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_getsockopt() do all of the work */

  ret = psock_getsockopt(psock, level, option, value, value_len);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
