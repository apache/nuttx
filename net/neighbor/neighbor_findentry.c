/****************************************************************************
 * net/neighbor/neighbor_findentry.c
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
#include <debug.h>

#include "neighbor/neighbor.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_findentry
 *
 * Description:
 *   Find an entry in the Neighbor Table.  This interface is internal to
 *   the neighbor implementation; Consider using neighbor_lookup() instead;
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address to use in the lookup;
 *
 * Returned Value:
 *   The Neighbor Table entry corresponding to the IPv6 address;  NULL is
 *   returned if there is no matching entry in the Neighbor Table.
 *
 ****************************************************************************/

FAR struct neighbor_entry_s *neighbor_findentry(const net_ipv6addr_t ipaddr)
{
  int i;

  for (i = 0; i < CONFIG_NET_IPv6_NCONF_ENTRIES; ++i)
    {
      FAR struct neighbor_entry_s *neighbor = &g_neighbors[i];

      if (net_ipv6addr_cmp(neighbor->ne_ipaddr, ipaddr))
        {
          neighbor_dumpentry("Entry found", neighbor);
          return neighbor;
        }
    }

  neighbor_dumpipaddr("Not found", ipaddr);
  return NULL;
}
