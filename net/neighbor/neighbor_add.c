/****************************************************************************
 * net/neighbor/neighbor_add.c
 *
 *   Copyright (C) 2007-2009, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * A leverage of logic from uIP which also has a BSD style license
 *
 *   Copyright (c) 2006, Swedish Institute of Computer Science.  All rights
 *     reserved.
 *   Author: Adam Dunkels <adam@sics.se>
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

#include "neighbor/neighbor.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_add
 *
 * Description:
 *   Add the new address association to the Neighbor Table (if it is not
 *   already there).
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address of the mapping.
 *   addr   - The link layer address of the mapping
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_add(FAR net_ipv6addr_t ipaddr, FAR struct neighbor_addr_s *addr)
{
  uint8_t oldest_time;
  int     oldest_ndx;
  int     i;

  nllvdbg("Add neighbor: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]),
          ntohs(ipaddr[3]), ntohs(ipaddr[4]), ntohs(ipaddr[5]),
          ntohs(ipaddr[6]), ntohs(ipaddr[7]));
  nllvdbg("  at: %02x:%02x:%02x:%02x:%02x:%02x\n",
          addr->na_addr.ether_addr_octet[0],
          addr->na_addr.ether_addr_octet[1],
          addr->na_addr.ether_addr_octet[2],
          addr->na_addr.ether_addr_octet[3],
          addr->na_addr.ether_addr_octet[4],
          addr->na_addr.ether_addr_octet[5]);

  /* Find the first unused entry or the oldest used entry. */

  oldest_time = 0;
  oldest_ndx  = 0;

  for (i = 0; i < CONFIG_NET_IPv6_NCONF_ENTRIES; ++i)
    {
      if (g_neighbors[i].ne_time == NEIGHBOR_MAXTIME)
        {
          oldest_ndx = i;
          break;
        }

      if (net_ipv6addr_cmp(g_neighbors[i].ne_ipaddr, ipaddr))
        {
          oldest_ndx = i;
          break;
        }

      if (g_neighbors[i].ne_time > oldest_time)
        {
          oldest_ndx = i;
          oldest_time = g_neighbors[i].ne_time;
        }
    }

  /* Use the oldest or first free entry (either pointed to by the
   * "oldest_ndx" variable).
   */

  g_neighbors[oldest_ndx].ne_time = 0;
  net_ipv6addr_copy(g_neighbors[oldest_ndx].ne_ipaddr, ipaddr);
  memcpy(&g_neighbors[oldest_ndx].ne_addr, addr, sizeof(struct neighbor_addr_s));
}
