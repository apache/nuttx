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
#include <nuttx/net/netdev.h>
#include <netdev/netdev.h>

#include "socket/socket.h"
#include "utils/utils.h"

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

  if (!value)
    {
      return -EFAULT;
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
        }
        break;

      case SO_BROADCAST:  /* Permits sending of broadcast messages */
      case SO_DEBUG:      /* Enables recording of debugging information */
      case SO_DONTROUTE:  /* Requests outgoing messages bypass standard routing */
      case SO_KEEPALIVE:  /* Verifies TCP connections active by enabling the
                           * periodic transmission of probes */
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

      /* There options are only valid when used with getopt */

      case SO_ACCEPTCONN: /* Reports whether socket listening is enabled */
      case SO_ERROR:      /* Reports and clears error status. */
      case SO_TYPE:       /* Reports the socket type */
        return -EINVAL;

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
  int ret = -ENOPROTOOPT;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Perform the socket interface operation */

  if (psock->s_sockif->si_setsockopt != NULL)
    {
      ret = psock->s_sockif->si_setsockopt(psock, level, option,
                                           value, value_len);
    }

  /* Try socket level if the socket interface operation is not available */

  if (ret == -ENOPROTOOPT && level == SOL_SOCKET)
    {
      ret = psock_socketlevel_option(psock, option, value, value_len);
    }

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

  ret = sockfd_socket(sockfd, &psock);

  /* Then let psock_setockopt() do all of the work */

  if (ret == OK)
    {
      ret = psock_setsockopt(psock, level, option, value, value_len);
    }

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
