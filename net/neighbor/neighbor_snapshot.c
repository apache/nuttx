/****************************************************************************
 * net/neighbor/neighbor_snapshot.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
          memcpy(&snapshot[ncopied], neighbor, sizeof(struct neighbor_entry_s));
          ncopied++;
        }
    }

  /* Return the number of entries copied into the user buffer */

  return ncopied;
}

#endif /* CONFIG_NETLINK_ROUTE */
