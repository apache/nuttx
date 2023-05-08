/****************************************************************************
 * net/inet/ipv6_getsockopt.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/net.h>

#include "mld/mld.h"
#include "inet/inet.h"
#include "udp/udp.h"

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_getsockopt
 *
 * Description:
 *   ipv6_getsockopt() sets the IPv6-protocol socket option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of IPv6 protocol
 *   socket options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.
 *
 ****************************************************************************/

int ipv6_getsockopt(FAR struct socket *psock, int option,
                    FAR void *value, FAR socklen_t *value_len)
{
  int ret;

  ninfo("option: %d\n", option);

  net_lock();
  switch (option)
    {
      case IPV6_TCLASS:
        {
          FAR struct socket_conn_s *conn = psock->s_conn;

          *(FAR uint8_t *)value = conn->s_tclass;
          *value_len = 1;
          ret = OK;
        }
        break;

      default:
        nerr("ERROR: Unrecognized IPv6 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
}

#endif /* CONFIG_NET_IPv6 */
