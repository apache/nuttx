/****************************************************************************
 * net/inet/ipv4_getsockopt.c
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

#include <nuttx/net/net.h>

#include <netinet/in.h>

#include "netfilter/iptables.h"

#ifdef CONFIG_NET_IPv4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_getsockopt
 *
 * Description:
 *   ipv4_getsockopt() retrieve the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   See <netinet/in.h> for the a complete list of values of IPv4 protocol
 *   socket options.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int ipv4_getsockopt(FAR struct socket *psock, int option,
                    FAR void *value, FAR socklen_t *value_len)
{
  int ret;

  ninfo("option: %d\n", option);

  net_lock();
  switch (option)
    {
#ifdef CONFIG_NET_IPTABLES
      case IPT_SO_GET_INFO:
      case IPT_SO_GET_ENTRIES:
        ret = ipt_getsockopt(psock, option, value, value_len);
        break;
#endif

      case IP_TOS:
        {
          FAR struct socket_conn_s *conn = psock->s_conn;

          *(FAR uint8_t *)value = conn->s_tos;
          *value_len = 1;
          ret = OK;
        }
        break;

      default:
        nerr("ERROR: Unrecognized IPv4 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
}

#endif /* CONFIG_NET_IPv4 */
