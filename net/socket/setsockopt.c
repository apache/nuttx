/****************************************************************************
 * net/socket/setsockopt.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <arch/irq.h>

#include <nuttx/net/net.h>
#include <netdev/netdev.h>

#include "socket/socket.h"
#include "inet/inet.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "usrsock/usrsock.h"
#include "utils/utils.h"
#include "can/can.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_socketlevel_option
 *
 * Description:
 *   psock_socketlevel_option() sets the socket-level option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <sys/socket.h> a complete list of values for the socket level
 *   'option' argument.
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

static int psock_socketlevel_option(FAR struct socket *psock, int option,
                                    FAR const void *value,
                                    socklen_t value_len)
{
  FAR struct socket_conn_s *conn = psock->s_conn;

  /* Verify that the socket option if valid (but might not be supported ) */

  if (!_SO_SETVALID(option) || !value)
    {
      return -EINVAL;
    }

  /* Process the options always handled locally */

  switch (option)
    {
      /* The following options take a pointer to an integer boolean value.
       * We will blindly set the bit here although the implementation
       * is outside of the scope of setsockopt.
       */

      case SO_RCVTIMEO:
      case SO_SNDTIMEO:
        {
          FAR struct timeval *tv = (FAR struct timeval *)value;
          socktimeo_t timeo;

          /* Verify that option is the size of an 'struct timeval'. */

          if (tv == NULL || value_len != sizeof(struct timeval))
            {
              return -EINVAL;
            }

          /* Get the timeout value.  Any microsecond remainder will be
           * forced to the next larger, whole decisecond value.
           */

          timeo = (socktimeo_t)net_timeval2dsec(tv, TV2DS_CEIL);

          /* Save the timeout value */

          if (option == SO_RCVTIMEO)
            {
              conn->s_rcvtimeo = timeo;
            }
          else
            {
              conn->s_sndtimeo = timeo;
            }

          /* Set/clear the corresponding enable/disable bit */

          if (timeo)
            {
              _SO_CLROPT(conn->s_options, option);
            }
          else
            {
              _SO_SETOPT(conn->s_options, option);
            }

          return OK;
        }
    }

#ifdef CONFIG_NET_USRSOCK
    if (psock->s_type == SOCK_USRSOCK_TYPE)
      {
        return -ENOPROTOOPT;
      }
#endif

  switch (option)
    {
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
          int setting;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          setting = *(FAR int *)value;

          /* Lock the network so that we have exclusive access to the socket
           * options.
           */

           net_lock();

          /* Set or clear the option bit */

          if (setting)
            {
              _SO_SETOPT(conn->s_options, option);
            }
          else
            {
              _SO_CLROPT(conn->s_options, option);
            }

          net_unlock();
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
        return tcp_setsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_SOLINGER
      case SO_LINGER:  /* Lingers on a close() if data is present */
        {
          FAR struct linger *setting;

          /* Verify that option is at least the size of an 'struct linger'. */

          if (value_len < sizeof(FAR struct linger))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          setting = (FAR struct linger *)value;

          /* Lock the network so that we have exclusive access to the socket
           * options.
           */

          net_lock();

          /* Set or clear the linger option bit and linger time
           * (in deciseconds)
           */

          if (setting->l_onoff)
            {
              _SO_SETOPT(conn->s_options, option);
              conn->s_linger = 10 * setting->l_linger;
            }
          else
            {
              _SO_CLROPT(conn->s_options, option);
              conn->s_linger = 0;
            }

          net_unlock();
        }
        break;
#endif

#ifdef CONFIG_NET_TIMESTAMP
      case SO_TIMESTAMP:  /* Generates a timestamp for each incoming packet */
        {
          /* Verify that option is at least the size of an integer. */

          if (value_len < sizeof(FAR int32_t))
            {
              return -EINVAL;
            }

          /* Lock the network so that we have exclusive access to the socket
           * options.
           */

          net_lock();

          conn->s_timestamp = *((FAR int32_t *)value);

          net_unlock();
        }
        break;
#endif

#if CONFIG_NET_RECV_BUFSIZE > 0
      case SO_RCVBUF:     /* Sets receive buffer size */
        {
          int buffersize;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          buffersize = *(FAR int *)value;

          if (buffersize < 0)
            {
              return -EINVAL;
            }

          net_lock();

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp;

              tcp = (FAR struct tcp_conn_s *)conn;

              /* Save the receive buffer size */

              tcp->rcv_bufs = buffersize;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp;

              udp = (FAR struct udp_conn_s *)conn;

              /* Save the receive buffer size */

              udp->rcvbufs = buffersize;
            }
          else
#endif
            {
              net_unlock();
              return -ENOPROTOOPT;
            }

          net_unlock();

          break;
        }
#endif

#if CONFIG_NET_SEND_BUFSIZE > 0
      case SO_SNDBUF:     /* Sets send buffer size */
        {
          int buffersize;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              return -EINVAL;
            }

          /* Get the value.  Is the option being set or cleared? */

          buffersize = *(FAR int *)value;

          if (buffersize < 0)
            {
              return -EINVAL;
            }

          net_lock();

#if defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_TCP_NO_STACK)
          if (psock->s_type == SOCK_STREAM)
            {
              FAR struct tcp_conn_s *tcp;

              tcp = (FAR struct tcp_conn_s *)conn;

              /* Save the send buffer size */

              tcp->snd_bufs = buffersize;
            }
          else
#endif
#if defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_UDP_NO_STACK)
          if (psock->s_type == SOCK_DGRAM)
            {
              FAR struct udp_conn_s *udp;

              udp = (FAR struct udp_conn_s *)conn;

              /* Save the send buffer size */

              udp->sndbufs = buffersize;
            }
          else
#endif
            {
              net_unlock();
              return -ENOPROTOOPT;
            }

          net_unlock();

          break;
        }
#endif

#ifdef CONFIG_NET_BINDTODEVICE
      /* Handle the SO_BINDTODEVICE socket-level option.
       *
       * NOTE: this option makes sense for UDP sockets trying to broadcast
       * while their local address is not set, eg, with DHCP requests.
       * The problem is that we are not able to determine the interface to be
       * used for sending packets when multiple interfaces do not have a
       * local address yet. This option can be used to "force" the interface
       * used to send the UDP traffic in this connection. Note that it does
       * NOT only apply to broadcast packets.
       */

      case SO_BINDTODEVICE:  /* Bind socket to a specific network device */
        {
          FAR struct net_driver_s *dev;

          /* Check if we are are unbinding the socket */

          if (value == NULL || value_len == 0 ||
             (value_len > 0 && ((FAR char *)value)[0] == 0))
            {
              conn->s_boundto = 0;  /* This interface is no longer bound */
              break;
            }

          /* No, we are binding a socket to the interface
           * Find the interface device with this name.
           */

          dev = netdev_findbyname(value);
          if (dev == NULL)
            {
              return -ENODEV;
            }

          /* Bind the socket to the interface */

          DEBUGASSERT(dev->d_ifindex > 0 &&
                      dev->d_ifindex <= MAX_IFINDEX);
          conn->s_boundto = dev->d_ifindex;

          break;
        }
#endif

      /* The following are not yet implemented */

      case SO_RCVLOWAT:   /* Sets the minimum number of bytes to input */
      case SO_SNDLOWAT:   /* Sets the minimum number of bytes to output */

      /* There options are only valid when used with getopt */

      case SO_ACCEPTCONN: /* Reports whether socket listening is enabled */
      case SO_ERROR:      /* Reports and clears error status. */
      case SO_TYPE:       /* Reports the socket type */

      default:
        return -ENOPROTOOPT;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_setsockopt
 *
 * Description:
 *   psock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket specified by the
 *   'psock' argument.
 *
 *   The 'level' argument specifies the protocol level of the option.  To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the socket level
 *   'option' argument.
 *
 *   Protocol level options, such as SOL_TCP, are defined in
 *   protocol-specific header files, for example include/netinet/tcp.h
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error:
 *
 *   EDOM
 *     The send and receive timeout values are too big to fit into the
 *     timeout fields in the socket structure.
 *   EINVAL
 *     The specified option is invalid at the specified socket 'level' or the
 *     socket has been shut down.
 *   EISCONN
 *     The socket is already connected, and a specified option cannot be set
 *     while the socket is connected.
 *   ENOPROTOOPT
 *     The 'option' is not supported by the protocol.
 *   ENOTSOCK
 *     The 'sockfd' argument does not refer to a socket.
 *   ENOMEM
 *     There was insufficient memory available for the operation to complete.
 *   ENOBUFS
 *    Insufficient resources are available in the system to complete the
 *    call.
 *
 ****************************************************************************/

int psock_setsockopt(FAR struct socket *psock, int level, int option,
                     FAR const void *value, socklen_t value_len)
{
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Handle setting of the socket option according to the level at which
   * option should be applied.
   */

  switch (level)
    {
      case SOL_SOCKET: /* Socket-level options (see include/sys/socket.h) */
        ret = psock_socketlevel_option(psock, option, value, value_len);
        break;

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
      case IPPROTO_TCP:/* TCP protocol socket options (see include/netinet/tcp.h) */
        ret = tcp_setsockopt(psock, option, value, value_len);
        break;
#endif

#ifdef CONFIG_NET_UDPPROTO_OPTIONS
      case IPPROTO_UDP:/* UDP protocol socket options (see include/netinet/udp.h) */
        ret = udp_setsockopt(psock, option, value, value_len);
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case IPPROTO_IP:/* TCP protocol socket options (see include/netinet/in.h) */
        ret = ipv4_setsockopt(psock, option, value, value_len);
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case IPPROTO_IPV6:/* TCP protocol socket options (see include/netinet/in.h) */
        ret = ipv6_setsockopt(psock, option, value, value_len);
        break;
#endif

#ifdef CONFIG_NET_CANPROTO_OPTIONS
      case SOL_CAN_RAW:   /* CAN protocol socket options (see include/netpacket/can.h) */
        ret = can_setsockopt(psock, option, value, value_len);
        break;
#endif

      default:         /* The provided level is invalid */
        ret = -ENOPROTOOPT;
        break;
    }

#ifdef CONFIG_NET_USRSOCK
  /* Try usrsock further if the protocol not available */

  if (ret == -ENOPROTOOPT && psock->s_type == SOCK_USRSOCK_TYPE)
    {
      ret = usrsock_setsockopt(psock->s_conn, level,
                               option, value, value_len);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: setsockopt
 *
 * Description:
 *   setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket associated with the
 *   file descriptor specified by the 'sockfd' argument.
 *
 *   The 'level' argument specifies the protocol level of the option.  To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the socket level
 *   'option' argument.
 *
 *   Protocol level options, such as SOL_TCP, are defined in
 *   protocol-specific header files, for example include/netinet/tcp.h
 *
 * Input Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   0 on success; -1 on failure
 *
 *   EBADF
 *     The 'sockfd' argument is not a valid socket descriptor.
 *   EDOM
 *     The send and receive timeout values are too big to fit into the
 *     timeout fields in the socket structure.
 *   EINVAL
 *     The specified option is invalid at the specified socket 'level' or the
 *     socket has been shut down.
 *   EISCONN
 *     The socket is already connected, and a specified option cannot be set
 *     while the socket is connected.
 *   ENOPROTOOPT
 *     The 'option' is not supported by the protocol.
 *   ENOTSOCK
 *     The 'sockfd' argument does not refer to a socket.
 *   ENOMEM
 *     There was insufficient memory available for the operation to complete.
 *   ENOBUFS
 *     Insufficient resources are available in the system to complete the
 *     call.
 *
 ****************************************************************************/

int setsockopt(int sockfd, int level, int option, const void *value,
               socklen_t value_len)
{
  FAR struct socket *psock;
  int ret;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_setockopt() do all of the work */

  ret = psock_setsockopt(psock, level, option, value, value_len);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
