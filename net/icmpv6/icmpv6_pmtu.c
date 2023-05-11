/****************************************************************************
 * net/icmpv6/icmpv6_pmtu.c
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

#include <nuttx/clock.h>

#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct icmpv6_pmtu_entry
g_icmpv6_pmtu_entry[CONFIG_NET_ICMPv6_PMTU_ENTRIES];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_find_pmtu_entry
 *
 * Description:
 *   Search for a ipv6 pmtu entry
 *
 * Parameters:
 *   destipaddr   the IPv6 address of the destination
 *
 * Return:
 *   not null is success; null is failure
 ****************************************************************************/

FAR struct icmpv6_pmtu_entry *
icmpv6_find_pmtu_entry(net_ipv6addr_t destipaddr)
{
  clock_t now = clock_systime_ticks();
  int i;

  for (i = 0; i < CONFIG_NET_ICMPv6_PMTU_ENTRIES; i++)
    {
      if (g_icmpv6_pmtu_entry[i].pmtu == 0)
        {
          continue;
        }

      if (net_ipv6addr_cmp(destipaddr, g_icmpv6_pmtu_entry[i].daddr))
        {
          g_icmpv6_pmtu_entry[i].time = now;
          return &g_icmpv6_pmtu_entry[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: icmpv6_add_pmtu_entry
 *
 * Description:
 *   Create a new ipv6 destination cache entry. If no unused entry is found,
 *   will recycle oldest entry
 *
 * Parameters:
 *   destipaddr   the IPv6 address of the destination
 *   mtu          MTU
 *
 * Return:
 *   void
 ****************************************************************************/

void icmpv6_add_pmtu_entry(net_ipv6addr_t destipaddr, int mtu)
{
  clock_t now = clock_systime_ticks();
  int j = 0;
  int i;

  for (i = 0; i < CONFIG_NET_ICMPv6_PMTU_ENTRIES; i++)
    {
      if (g_icmpv6_pmtu_entry[i].pmtu == 0 ||
          (sclock_t)(now - g_icmpv6_pmtu_entry[i].time) >=
          SEC2TICK(CONFIG_NET_ICMPv6_PMTU_TIMEOUT * 60))
        {
          j = i;
          break;
        }

      if ((sclock_t)(g_icmpv6_pmtu_entry[i].time -
          g_icmpv6_pmtu_entry[j].time) < 0)
        {
          j = i;
        }
    }

  net_ipv6addr_copy(g_icmpv6_pmtu_entry[j].daddr, destipaddr);
  g_icmpv6_pmtu_entry[j].pmtu = mtu;
  g_icmpv6_pmtu_entry[j].time = now;
}
