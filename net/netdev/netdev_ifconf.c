/****************************************************************************
 * net/netdev/netdev_ifconf.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <nuttx/net/netdev.h>

#include "inet/inet.h"
#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The form of information passed with netdev_foreach() callback */

#ifdef CONFIG_NET_IPv4
struct ifconf_ipv4_info_s
{
  size_t bufsize;
  FAR struct ifconf *ifc;
};
#endif

#ifdef CONFIG_NET_IPv6
struct ifconf_ipv6_info_s
{
  size_t bufsize;
  FAR struct lifconf *lifc;
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ifconf_ipv4_callback
 *
 * Description:
 *   Callback from netdev_foreach() that does the real implementation of
 *   netdev_ipv4_ifconf().
 *
 * Input Parameters:
 *   dev - The network device for this callback.
 *   arg - User callback argument
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int ifconf_ipv4_callback(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct ifconf_ipv4_info_s *info = (FAR struct ifconf_ipv4_info_s *)arg;
  FAR struct ifconf *ifc;

  DEBUGASSERT(dev != NULL && info != NULL && info->ifc != NULL);
  ifc = info->ifc;

  /* Check if this adapter has an IPv4 address assigned and is in the UP
   * state.
   */

  if (!net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY) &&
      (dev->d_flags & IFF_UP) != 0)
    {
      /* Check if we would exceed the buffer space provided by the caller.
       * NOTE: A common usage model is:
       *
       * 1) Provide no buffer with the first SIOCGIFCONF command.  In this
       *    case, only the size of the buffer needed for all of IPv4
       *    capable interface information is returned.
       * 2) Allocate a buffer of that size
       * 3) Then use the allocated buffer to receive all of the IPv4
       *    interface information on the second SIOCGIFCONF command.
       *
       * CAUTION: The number of IPv6 capable interface may change between
       * calls.  It is the callers responsibility to behave sanely in such
       * cases.
       */

      if (ifc->ifc_len + sizeof(struct ifreq) <= info->bufsize)
        {
          FAR struct ifreq *req =
            (FAR struct ifreq *)&ifc->ifc_buf[ifc->ifc_len];
          FAR struct sockaddr_in *inaddr =
            (FAR struct sockaddr_in *)&req->ifr_addr;

          /* There is space for information about another adapter.  Within
           * each ifreq structure, ifr_name will receive the interface name
           * and ifr_addr the address.  The actual number of bytes
           * transferred is returned in ifc_len.
           */

          strlcpy(req->ifr_name, dev->d_ifname, IFNAMSIZ);

          inaddr->sin_family = AF_INET;
          inaddr->sin_port   = 0;
          net_ipv4addr_copy(inaddr->sin_addr.s_addr, dev->d_ipaddr);
          memset(inaddr->sin_zero, 0, sizeof(inaddr->sin_zero));
        }

      /* Increment the size of the buffer in any event */

      ifc->ifc_len += sizeof(struct ifreq);
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: ifconf_ipv6_callback
 *
 * Description:
 *   Callback from netdev_foreach() that does the real implementation of
 *   netdev_ipv6_ifconf().
 *
 * Input Parameters:
 *   dev - The network device for this callback.
 *   arg - User callback argument
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int ifconf_ipv6_callback(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct ifconf_ipv6_info_s *info = (FAR struct ifconf_ipv6_info_s *)arg;
  FAR struct lifconf *lifc;

  DEBUGASSERT(dev != NULL && info != NULL && info->lifc != NULL);
  lifc = info->lifc;

  /* Check if this adapter has an IPv6 address assigned and is in the UP
   * state.
   */

  if (!net_ipv6addr_cmp(dev->d_ipv6addr, g_ipv6_unspecaddr) &&
      (dev->d_flags & IFF_UP) != 0)
    {
      /* Check if we would exceed the buffer space provided by the caller.
       * NOTE: A common usage model is:
       *
       * 1) Provide no buffer with the first SIOCGLIFCONF command.  In this
       *    case, only the size of the buffer needed for all of IPv6
       *    capable interface information is returned.
       * 2) Allocate a buffer of that size
       * 3) Then use the allocated buffer to receive all of the IPv6
       *    interface information on the second SIOCGLIFCONF command.
       *
       * CAUTION: The number of IPv6 capable interface may change between
       * calls.  It is the callers responsibility to behave sanely in such
       * cases.
       */

      if (lifc->lifc_len + sizeof(struct lifreq) <= info->bufsize)
        {
          FAR struct lifreq *req =
            (FAR struct lifreq *)&lifc->lifc_buf[lifc->lifc_len];
          FAR struct sockaddr_in6 *inaddr =
            (FAR struct sockaddr_in6 *)&req->lifr_addr;

          /* There is space for information about another adapter.  Within
           * each ifreq structure, lifr_name will receive the interface
           * name and lifr_addr the address.  The actual number of bytes
           * transferred is returned in lifc_len.
           */

          strlcpy(req->lifr_name, dev->d_ifname, IFNAMSIZ);

          inaddr->sin6_family = AF_INET6;
          inaddr->sin6_port   = 0;
          net_ipv6addr_copy(inaddr->sin6_addr.s6_addr16, dev->d_ipv6addr);
        }

      /* Increment the size of the buffer in any event */

      lifc->lifc_len += sizeof(struct lifreq);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ipv4_ifconf
 *
 * Description:
 *   Return the IPv4 configuration of each network adapter that (1) has
 *   and IPv4 address assigned and (2) is in the UP state
 *
 * Input Parameters:
 *   ifc - A reference to the instance of struct ifconf in which to return
 *         the information.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *  The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int netdev_ipv4_ifconf(FAR struct ifconf *ifc)
{
  struct ifconf_ipv4_info_s info;

  info.bufsize = ifc->ifc_len;
  info.ifc     = ifc;
  ifc->ifc_len = 0;

  return netdev_foreach(ifconf_ipv4_callback, (FAR void *)&info);
}
#endif

/****************************************************************************
 * Name: netdev_ipv6_ifconf
 *
 * Description:
 *   Return the IPv6 configuration of each network adapter that (1) has
 *   and IPv6 address assigned and (2) is in the UP state.
 *
 * Input Parameters:
 *   lifc - A reference to the instance of struct lifconf in which to return
 *          the information.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *  The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int netdev_ipv6_ifconf(FAR struct lifconf *lifc)
{
  struct ifconf_ipv6_info_s info;

  info.bufsize   = lifc->lifc_len;
  info.lifc      = lifc;
  lifc->lifc_len = 0;

  return netdev_foreach(ifconf_ipv6_callback, (FAR void *)&info);
}
#endif
