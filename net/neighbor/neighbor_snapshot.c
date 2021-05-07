/****************************************************************************
 * net/neighbor/neighbor_snapshot.c
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

#include <nuttx/net/ip.h>

#include "inet/inet.h"
#include "neighbor/neighbor.h"

#ifdef CONFIG_NETLINK_ROUTE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_snapshot
 *
 * Description:
 *   Take a snapshot of the current state of the Neighbor table.
 *
 * Input Parameters:
 *   snapshot  - Location to return the Neighbor table copy
 *   nentries  - The size of the user provided 'dest' in entries, each of
 *               size sizeof(struct arp_entry_s)
 *
 * Returned Value:
 *   On success, the number of entries actually copied is returned.  Unused
 *   entries are not returned.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

unsigned int neighbor_snapshot(FAR struct neighbor_entry_s *snapshot,
                               unsigned int nentries)
{
  unsigned int ncopied;
  int i;

  /* Copy all non-empty entries in the Neighbor table. */

  for (i = 0, ncopied = 0;
       nentries > ncopied && i < CONFIG_NET_IPv6_NCONF_ENTRIES;
       i++)
    {
      FAR struct neighbor_entry_s *neighbor = &g_neighbors[i];

      /* An unused entry table entry will be nullified.  In particularly,
       * the Neighbor IP address will be all zero (i.e., the unspecified
       * IPv6 address).
       */

      if (!net_ipv6addr_cmp(neighbor->ne_ipaddr, g_ipv6_unspecaddr))
        {
          memcpy(&snapshot[ncopied], neighbor,
                 sizeof(struct neighbor_entry_s));
          ncopied++;
        }
    }

  /* Return the number of entries copied into the user buffer */

  return ncopied;
}

#endif /* CONFIG_NETLINK_ROUTE */
