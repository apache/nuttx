/****************************************************************************
 * net/netdev/netdev_ipv6.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/net/netdev.h>

#include "inet/inet.h"
#include "netdev/netdev.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Defined in Section 2.7 of RFC4291 */

#define IPv6_SCOPE_INTERFACE_LOCAL    0x1
#define IPv6_SCOPE_LINK_LOCAL         0x2
#define IPv6_SCOPE_ADMIN_LOCAL        0x4
#define IPv6_SCOPE_SITE_LOCAL         0x5
#define IPv6_SCOPE_ORGANIZATION_LOCAL 0x8
#define IPv6_SCOPE_GLOBAL             0xe

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ipv6_mcastmac
 *
 * Description:
 *   Given an IPv6 address (in network order), create a IPv6 multicast MAC
 *   address for ICMPv6 Neighbor Solicitation message.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void netdev_ipv6_mcastmac(const net_ipv6addr_t addr, FAR uint8_t *mac)
{
  FAR const uint8_t *ipaddr8 = (FAR const uint8_t *)addr;

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;
  mac[2] = 0xff;
  mac[3] = ipaddr8[13];  /* Bits: 104-111 */
  mac[4] = ipaddr8[14];  /* Bits: 112-119 */
  mac[5] = ipaddr8[15];  /* Bits: 120-127 */
}
#endif

/****************************************************************************
 * Name: netdev_ipv6_get_scope
 ****************************************************************************/

#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
static uint8_t netdev_ipv6_get_scope(const net_ipv6addr_t addr)
{
  if (net_is_addr_mcast(addr))
    {
      /* As defined in Section 2.7 of RFC4291:
       * |   8    |  4 |  4 |                  112 bits                   |
       * +------ -+----+----+---------------------------------------------+
       * |11111111|flgs|scop|                  group ID                   |
       * +--------+----+----+---------------------------------------------+
       */

      return NTOHS(addr[0]) & 0x000f;
    }

  if (net_is_addr_linklocal(addr))
    {
      return IPv6_SCOPE_LINK_LOCAL;
    }

  if (net_is_addr_sitelocal(addr))
    {
      return IPv6_SCOPE_SITE_LOCAL;
    }

  return IPv6_SCOPE_GLOBAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ipv6_add/del
 *
 * Description:
 *   Add or delete an IPv6 address on the network device
 *
 * Returned Value:
 *   OK             - Success
 *   -EINVAL        - Invalid prefix length
 *   -EADDRNOTAVAIL - Delete on non-existent address
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

int netdev_ipv6_add(FAR struct net_driver_s *dev, const net_ipv6addr_t addr,
                    unsigned int preflen)
{
  FAR struct netdev_ifaddr6_s *ifaddr = &dev->d_ipv6[0];
#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
  uint8_t scope;
  int i;
#endif

  /* Verify the prefix length */

  if (preflen > 128)
    {
      return -EINVAL;
    }

#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
  /* Avoid duplicate address. */

  ifaddr = netdev_ipv6_lookup(dev, addr, false);
  if (ifaddr != NULL)
    {
      /* Check if net mask is the same. */

      if (net_ipv6_mask2pref(ifaddr->mask) == preflen)
        {
          nwarn("WARNING: Trying to add same IPv6 address on net device! "
                "%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x/%d\n",
                NTOHS(addr[0]), NTOHS(addr[1]), NTOHS(addr[2]),
                NTOHS(addr[3]), NTOHS(addr[4]), NTOHS(addr[5]),
                NTOHS(addr[6]), NTOHS(addr[7]), preflen);
          return -EEXIST;
        }

      /* Not exactly the same, update the net mask.
       * REVISIT: Currently try to keep logic same as previous, which always
       *          allows to override the address. But not sure if it's good.
       */

      net_ipv6_pref2mask(ifaddr->mask, preflen);
      return OK;
    }

  /* Now we start to find a proper slot to put this address. */

  ifaddr = &dev->d_ipv6[0]; /* Set default to a valid address. */
  scope = netdev_ipv6_get_scope(addr);

  for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
    {
      FAR struct netdev_ifaddr6_s *current = &dev->d_ipv6[i];

      /* Select empty address. */

      if (net_ipv6addr_cmp(current->addr, g_ipv6_unspecaddr))
        {
          ifaddr = current;
          break;
        }

      /* Select address with same scope. */

      if (netdev_ipv6_get_scope(current->addr) == scope)
        {
          ifaddr = current;
          continue; /* Good slot, but maybe we have empty slot later. */
        }
    }
#endif /* CONFIG_NETDEV_MULTIPLE_IPv6 */

  net_ipv6addr_copy(ifaddr->addr, addr);
  net_ipv6_pref2mask(ifaddr->mask, preflen);

  netdev_ipv6_addmcastmac(dev, addr);

  return OK;
}

int netdev_ipv6_del(FAR struct net_driver_s *dev, const net_ipv6addr_t addr,
                    unsigned int preflen)
{
  FAR struct netdev_ifaddr6_s *ifaddr;

  /* Verify the prefix length */

  if (preflen > 128)
    {
      return -EINVAL;
    }

  /* Find the matching address entry */

  ifaddr = netdev_ipv6_lookup(dev, addr, false);
  if (ifaddr == NULL)
    {
      /* The address does not exist on the device */

      return -EADDRNOTAVAIL;
    }

  if (net_ipv6_mask2pref(ifaddr->mask) != preflen)
    {
      /* Prefix length does not match, regard as not found (same as Linux) */

      return -EADDRNOTAVAIL;
    }

  /* Delete the address */

  net_ipv6addr_copy(ifaddr->addr, g_ipv6_unspecaddr);
  net_ipv6addr_copy(ifaddr->mask, g_ipv6_unspecaddr);

  netdev_ipv6_removemcastmac(dev, addr);

  return OK;
}

/****************************************************************************
 * Name: netdev_ipv6_addmcastmac/removemcastmac
 *
 * Description:
 *   Add / Remove an MAC address corresponds to the IPv6 address to / from
 *   the device's MAC filter table.
 *
 * Input Parameters:
 *   dev  - The device driver structure to be modified
 *   addr - The IPv6 address whose related MAC will be added or removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
void netdev_ipv6_addmcastmac(FAR struct net_driver_s *dev,
                             const net_ipv6addr_t addr)
{
  uint8_t mcastmac[ETHER_ADDR_LEN];

  if (net_ipv6addr_cmp(addr, g_ipv6_unspecaddr))
    {
      return;
    }

  if (dev->d_addmac != NULL)
    {
      netdev_ipv6_mcastmac(addr, mcastmac);
      ninfo("Add IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
            mcastmac[0], mcastmac[1], mcastmac[2],
            mcastmac[3], mcastmac[4], mcastmac[5]);
      dev->d_addmac(dev, mcastmac);
    }
}

void netdev_ipv6_removemcastmac(FAR struct net_driver_s *dev,
                                const net_ipv6addr_t addr)
{
  uint8_t mcastmac[ETHER_ADDR_LEN];
#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
  int i;
#endif

  if (net_ipv6addr_cmp(addr, g_ipv6_unspecaddr))
    {
      return;
    }

  if (dev->d_rmmac != NULL)
    {
      netdev_ipv6_mcastmac(addr, mcastmac);

#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
      /* Avoid removing mac needed by other addresses. */

      for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
        {
          FAR struct netdev_ifaddr6_s *current = &dev->d_ipv6[i];
          uint8_t currentmac[ETHER_ADDR_LEN];

          /* Skip empty address and target address */

          if (net_ipv6addr_cmp(current->addr, g_ipv6_unspecaddr) ||
              net_ipv6addr_cmp(current->addr, addr))
            {
              continue;
            }

          /* Generate multicast MAC for this address. */

          netdev_ipv6_mcastmac(current->addr, currentmac);

          /* We don't remove the MAC if any other IPv6 address needs it. */

          if (memcmp(currentmac, mcastmac, ETHER_ADDR_LEN) == 0)
            {
              return;
            }
        }
#endif /* CONFIG_NETDEV_MULTIPLE_IPv6 */

      ninfo("Remove IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
            mcastmac[0], mcastmac[1], mcastmac[2],
            mcastmac[3], mcastmac[4], mcastmac[5]);
      dev->d_rmmac(dev, mcastmac);
    }
}
#endif

/****************************************************************************
 * Name: netdev_ipv6_srcaddr/srcifaddr
 *
 * Description:
 *   Get the source IPv6 address (RFC6724) to use for transmitted packets.
 *   If we are responding to a received packet, use the destination address
 *   from that packet. If we are initiating communication, pick a local
 *   address that best matches the destination address.
 *
 * Input parameters:
 *   dev - Network device that packet is being transmitted from
 *   dst - Address to compare against when choosing local address.
 *
 * Returned Value:
 *   A pointer to a net_ipv6addr_t contained in net_driver_s is returned on
 *   success.  It will never be NULL, but can be an address containing
 *   g_ipv6_unspecaddr.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR const uint16_t *netdev_ipv6_srcaddr(FAR struct net_driver_s *dev,
                                        const net_ipv6addr_t dst)
{
  return netdev_ipv6_srcifaddr(dev, dst)->addr;
}

FAR const struct netdev_ifaddr6_s *
netdev_ipv6_srcifaddr(FAR struct net_driver_s *dev, const net_ipv6addr_t dst)
{
  FAR struct netdev_ifaddr6_s *best = &dev->d_ipv6[0]; /* Don't be NULL */
#ifdef CONFIG_NETDEV_MULTIPLE_IPv6
  uint8_t scope_dst  = netdev_ipv6_get_scope(dst);
  uint8_t scope_best = 0; /* All scope is larget than 0 */
  uint8_t pref_best  = 0;
  int i;

  for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
    {
      FAR struct netdev_ifaddr6_s *current = &dev->d_ipv6[i];
      uint8_t scope_cur;
      uint8_t pref_cur;

      /* Skip empty address */

      if (net_ipv6addr_cmp(current->addr, g_ipv6_unspecaddr))
        {
          continue;
        }

      /* Rule 1: Prefer same address */

      if (net_ipv6addr_cmp(dst, current->addr))
        {
          best = current;
          break;
        }

      scope_cur = netdev_ipv6_get_scope(current->addr);
      pref_cur  = net_ipv6_common_pref(current->addr, dst);

      /* Rule 2: Prefer appropriate scope */

      if (scope_cur != scope_best)
        {
          /* According to RFC6724:
           * If Scope(SA) < Scope(SB):
           *  If Scope(SA) < Scope(D), then prefer SB and otherwise prefer SA
           * If Scope(SB) < Scope(SA):
           *  If Scope(SB) < Scope(D), then prefer SA and otherwise prefer SB
           * Let Scope(SA)->Scope(cur), Scope(SB)->Scope(best) in our case.
           */

          if ((scope_cur < scope_best && scope_cur >= scope_dst) ||
              (scope_best < scope_cur && scope_best < scope_dst))
            {
              best       = current;
              scope_best = scope_cur;
              pref_best  = pref_cur;
            }

          continue;
        }

      /* Rule 3: Avoid deprecated and optimistic addresses
       *         [Not implemented: Need DAD & address type support]
       * Rule 4: Prefer home address
       *         [Not implemented: Need MIP6]
       * Rule 5: Prefer outgoing interface
       *         [Already satisfied: We already have the device]
       * Rule 6: Prefer matching label
       *         [Not implemented: Need policy table support]
       *         [Note: Neither lwIP nor Zephyr supports policy table yet]
       * Rule 7: Prefer temporary addresses
       *         [Not implemented: Need DAD & temporary addresses support]
       */

      /* Rule 8: Use longest matching prefix */

      if (pref_cur > pref_best)
        {
          best       = current;
          scope_best = scope_cur;
          pref_best  = pref_cur;
        }
    }
#endif /* CONFIG_NETDEV_MULTIPLE_IPv6 */

  return best;
}

/****************************************************************************
 * Name: netdev_ipv6_lladdr
 *
 * Description:
 *   Get the link-local address of the network device.
 *
 * Returned Value:
 *   A pointer to the link-local address is returned on success.
 *   NULL is returned if the address is not found on the device.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR const uint16_t *netdev_ipv6_lladdr(FAR struct net_driver_s *dev)
{
  int i;

  for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
    {
      FAR struct netdev_ifaddr6_s *ifaddr = &dev->d_ipv6[i];

      if (net_is_addr_linklocal(ifaddr->addr))
        {
          return ifaddr->addr;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: netdev_ipv6_lookup
 *
 * Description:
 *   Look up an IPv6 address in the network device's IPv6 addresses
 *
 * Input Parameters:
 *   dev     - The network device to use in the lookup
 *   addr    - The IPv6 address to be looked up
 *   maskcmp - If true, then the IPv6 address is compared to the network
 *             device's IPv6 addresses with mask compare.
 *             If false, then the IPv6 address should be exactly the same as
 *             the network device's IPv6 address.
 *
 * Returned Value:
 *   A pointer to the matching IPv6 address entry is returned on success.
 *   NULL is returned if the IPv6 address is not found in the device.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR struct netdev_ifaddr6_s *
netdev_ipv6_lookup(FAR struct net_driver_s *dev, const net_ipv6addr_t addr,
                   bool maskcmp)
{
  int i;

  for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
    {
      FAR struct netdev_ifaddr6_s *ifaddr = &dev->d_ipv6[i];

      /* Skip empty address */

      if (net_ipv6addr_cmp(ifaddr->addr, g_ipv6_unspecaddr))
        {
          continue;
        }

      /* Check if the address matches */

      if (maskcmp)
        {
          if (net_ipv6addr_maskcmp(addr, ifaddr->addr, ifaddr->mask))
            {
              return ifaddr;
            }
        }
      else
        {
          if (net_ipv6addr_cmp(addr, ifaddr->addr))
            {
              return ifaddr;
            }
        }
    }

  /* No match found */

  return NULL;
}

/****************************************************************************
 * Name: netdev_ipv6_foreach
 *
 * Description:
 *   Enumerate each IPv6 address on a network device.  This function will
 *   terminate when either (1) all addresses have been enumerated or (2) when
 *   a callback returns any non-zero value.
 *
 * Input Parameters:
 *   dev      - The network device
 *   callback - Will be called for each IPv6 address
 *   arg      - Opaque user argument passed to callback()
 *
 * Returned Value:
 *  Zero:     Enumeration completed
 *  Non-zero: Enumeration terminated early by callback
 *
 * Assumptions:
 *  The network is locked.
 *
 ****************************************************************************/

int netdev_ipv6_foreach(FAR struct net_driver_s *dev,
                        devif_ipv6_callback_t callback, FAR void *arg)
{
  int i;

  if (callback == NULL)
    {
      return OK;
    }

  for (i = 0; i < CONFIG_NETDEV_MAX_IPv6_ADDR; i++)
    {
      FAR struct netdev_ifaddr6_s *ifaddr = &dev->d_ipv6[i];

      if (!net_ipv6addr_cmp(ifaddr->addr, g_ipv6_unspecaddr))
        {
          int ret = callback(dev, ifaddr, arg);
          if (ret != 0) /* Stop on any error and return it */
            {
              return ret;
            }
        }
    }

  return OK;
}
