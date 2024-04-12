/****************************************************************************
 * net/udp/udp_setsockopt.c
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/udp.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "devif/devif.h"
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
  FAR struct udp_conn_s *conn;
  int ret = OK;

  conn = psock->s_conn;

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR:  Not a UDP socket\n");
      return -ENOTCONN;
    }

  /* Handle the UDP option */

  switch (option)
    {
#ifdef CONFIG_NET_UDP_OFFLOAD
      case UDP_SEGMENT:
        if (value_len < sizeof(int))
          {
            ret = -EINVAL;
          }
        else
          {
            int gso_size = *(FAR int *)value;
            if (gso_size > GSO_MAX_SIZE)
              {
                return -EINVAL;
              }

            conn->gso_size = gso_size;
          }
        break;
      case UDP_GRO:
        if (value_len < sizeof(int))
          {
            ret = -EINVAL;
          }
        else
          {
            conn->gro_enable = (bool)(*(FAR int *)value);
          }
        break;
#endif
      default:
        nerr("ERROR: Unrecognized UDP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_UDPPROTO_OPTIONS */
