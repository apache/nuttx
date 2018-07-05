/****************************************************************************
 * net/udp/udp_setsockopt.c
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

#include <netinet/udp.h>

#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "netdev/netdev.h"
#include "udp/udp.h"

#ifdef CONFIG_NET_UDPPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_setsockopt
 *
 * Description:
 *   udp_setsockopt() sets the UDP-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/udp.h> for the a complete list of values of UDP protocol
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

int udp_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len)
{
#ifdef CONFIG_NET_UDP_BINDTODEVICE
  /* Keep alive options are the only UDP protocol socket option currently
   * supported.
   */

  FAR struct udp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && psock->s_conn != NULL);
  conn = (FAR struct udp_conn_s *)psock->s_conn;

  /* All of the UDP protocol options apply only UDP sockets.  The sockets
   * do not have to be connected.. that might occur later with the KeepAlive
   * already configured.
   */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR:  Not a UDP socket\n");
      return -ENOTCONN;
    }

  /* Handle the UDP-protocol options */

  switch (option)
    {
#ifdef CONFIG_NET_UDP_BINDTODEVICE
      /* Handle the UDP_BINDTODEVICE socket-level option.
       *
       * NOTE: UDP_BINDTODEVICE is declared in linux as SO_BINDTODEVICE,
       * but this option only makes sense for UDP sockets trying to broadcast
       * while their local address is not set, eg, with DHCP requests.
       * The problem is that we are not able to determine the interface to be
       * used for sending packets when multiple interfaces do not have a local
       * address yet. This option can be used to "force" the interface used to
       * send the UDP traffic in this connection. Note that it does NOT only
       * apply to broadcast packets.
       */

      case UDP_BINDTODEVICE:  /* Bind socket to a specific network device */
        if (value == NULL || value_len == 0 ||
           (value_len > 0 && ((FAR char *)value)[0] == 0))
          {
            conn->boundto = 0;  /* This interface is no longer bound */
            ret = OK;
          }
        else
          {
            int ifindex;

            /* Get the interface index corresponding to the interface name */

            ifindex = netdev_nametoindex(value);
            if (ifindex >= 0)
              {
                DEBUGASSERT(ifindex > 0 && ifindex <= MAX_IFINDEX);
                conn->boundto = ifindex;
                ret = OK;
              }
            else
              {
                ret = ifindex;
              }
          }

        break;
#endif

      default:
        nerr("ERROR: Unrecognized UDP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
#else
  return -ENOPROTOOPT;
#endif /* CONFIG_NET_UDP_BINDTODEVICE */
}

#endif /* CONFIG_NET_UDPPROTO_OPTIONS */

