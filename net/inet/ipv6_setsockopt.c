/****************************************************************************
 * net/inet/ipv6_setsockopt.c
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

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_setsockopt
 *
 * Description:
 *   ipv6_setsockopt() sets the IPv6-protocol socket option specified by the
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
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int ipv6_setsockopt(FAR struct socket *psock, int option,
                    FAR const void *value, socklen_t value_len)
{
#ifdef CONFIG_NET_MLD
  int ret;

  ninfo("option: %d\n", option);

  /* Handle MLD-related socket options */

  net_lock();
  switch (option)
    {
      case IPV6_JOIN_GROUP:       /* Join a multicast group */
        {
          FAR const struct ipv6_mreq *mrec ;

          mrec = (FAR const struct ipv6_mreq *)value;
          if (mrec == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = mld_joingroup(mrec);
            }
        }
        break;

      case IPV6_LEAVE_GROUP:      /* Quit a multicast group */
        {
          FAR const struct ipv6_mreq *mrec ;

          mrec = (FAR const struct ipv6_mreq *)value;
          if (mrec == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              ret = mld_leavegroup(mrec);
            }
        }
        break;

      /* The following IPv6 socket options are defined, but not implemented */

      case IPV6_MULTICAST_HOPS:   /* Multicast hop limit */
      case IPV6_MULTICAST_IF:     /* Interface to use for outgoing multicast
                                   * packets */
      case IPV6_MULTICAST_LOOP:   /* Multicast packets are delivered back to
                                   * the local application */
      case IPV6_UNICAST_HOPS:     /* Unicast hop limit */
      case IPV6_V6ONLY:           /* Restrict AF_INET6 socket to IPv6
                                   * communications only */
        nwarn("WARNING: Unimplemented IPv6 option: %d\n", option);
        ret = -ENOSYS;
        break;

      default:
        nerr("ERROR: Unrecognized IPv6 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
#else
  return -ENOPROTOOPT;
#endif
}

#endif /* CONFIG_NET_IPv6 */
