/****************************************************************************
 * net/icmp/icmp_pmtu.c
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

#include <sys/time.h>
#include <nuttx/clock.h>

#include "icmp/icmp.h"
#include "utils/utils.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct icmp_pmtu_entry
g_icmp_pmtu_entry[CONFIG_NET_ICMP_PMTU_ENTRIES];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv4_find_pmtu_entry
 *
 * Description:
 *   Search for a ipv4 destination cache entry
 *
 * Parameters:
 *   destipaddr   the IPv4 address of the destination
 *
 * Return:
 *   not null is success; null is failure
 ****************************************************************************/

FAR struct icmp_pmtu_entry *icmpv4_find_pmtu_entry(in_addr_t destipaddr)
{
  clock_t now = clock_systime_ticks();
  int i;

  for (i = 0; i < CONFIG_NET_ICMP_PMTU_ENTRIES; i++)
    {
      if (g_icmp_pmtu_entry[i].pmtu == 0)
        {
          continue;
        }

      if (net_ipv4addr_cmp(destipaddr, g_icmp_pmtu_entry[i].daddr))
        {
          g_icmp_pmtu_entry[i].time = now;
          return &g_icmp_pmtu_entry[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: icmpv4_add_pmtu_entry
 *
 * Description:
 *   Create a new ipv4 destination cache entry. If no unused entry is found,
 *   will recycle oldest entry
 *
 * Parameters:
 *   destipaddr   the IPv4 address of the destination
 *   mtu          MTU
 *
 * Return:
 *   void
 ****************************************************************************/

void icmpv4_add_pmtu_entry(in_addr_t destipaddr, int mtu)
{
  clock_t now = clock_systime_ticks();
  int j = 0;
  int i;

  for (i = 0; i < CONFIG_NET_ICMP_PMTU_ENTRIES; i++)
    {
      if ((g_icmp_pmtu_entry[i].pmtu == 0) ||
          (sclock_t)(now - g_icmp_pmtu_entry[i].time) >=
          SEC2TICK(CONFIG_NET_ICMP_PMTU_TIMEOUT * 60))
        {
          j = i;
          break;
        }

      if ((sclock_t)(g_icmp_pmtu_entry[i].time -
          g_icmp_pmtu_entry[j].time) < 0)
        {
          j = i;
        }
    }

  net_ipv4addr_copy(g_icmp_pmtu_entry[j].daddr, destipaddr);
  g_icmp_pmtu_entry[j].pmtu = mtu;
  g_icmp_pmtu_entry[j].time = now;
}
