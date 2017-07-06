/****************************************************************************
 * net/neighbor/neighbor_dumpentry.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <nuttx/net/net.h>

#include "neighbor/neighbor.h"

#ifdef CONFIG_DEBUG_NET_INFO

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_dumpentry
 *
 * Description:
 *   Dump the conents of an entry Neighbor Table.
 *
 * Input Parameters:
 *   msg      - Message to print with the entry
 *   neighbor - The table entry to dump
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void neighbor_dumpentry(FAR const char *msg,
                        FAR struct neighbor_entry *neighbor)
{
  ninfo("%s: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        msg,
        ntohs(neighbor->ne_ipaddr[0]), ntohs(neighbor->ne_ipaddr[1]),
        ntohs(neighbor->ne_ipaddr[2]), ntohs(neighbor->ne_ipaddr[3]),
        ntohs(neighbor->ne_ipaddr[4]), ntohs(neighbor->ne_ipaddr[5]),
        ntohs(neighbor->ne_ipaddr[6]), ntohs(neighbor->ne_ipaddr[7]));

#ifdef CONFIG_NET_ETHERNET
#ifdef CONFIG_NET_6LOWPAN
  if (neighbor->ne_addr.u.na_lltype == NET_LL_ETHERNET)
#endif
    {
      ninfo("  at: %02x:%02x:%02x:%02x:%02x:%02x\n",
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[0],
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[1],
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[2],
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[3],
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[4],
           neighbor->ne_addr.u.na_ethernet.ether_addr_octet[5]);
    }
#endif
#ifdef CONFIG_NET_6LOWPAN
#ifdef CONFIG_NET_ETHERNET
  else
#endif
    {
#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
      ninfo("  at: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
            neighbor->ne_addr.u.na_sixlowpan.u8[0],
            neighbor->ne_addr.u.na_sixlowpan.u8[1],
            neighbor->ne_addr.u.na_sixlowpan.u8[2],
            neighbor->ne_addr.u.na_sixlowpan.u8[3],
            neighbor->ne_addr.u.na_sixlowpan.u8[4],
            neighbor->ne_addr.u.na_sixlowpan.u8[5],
            neighbor->ne_addr.u.na_sixlowpan.u8[6],
            neighbor->ne_addr.u.na_sixlowpan.u8[7]);
#else
      ninfo("  at: %02x:%02x\n",
            neighbor->ne_addr.u.na_sixlowpan.u8[0],
            neighbor->ne_addr.u.na_sixlowpan.u8[1]);
    }
#endif
#endif
}

/****************************************************************************
 * Name: neighbor_dumpipaddr
 *
 * Description:
 *   Dump an IP address.
 *
 * Input Parameters:
 *   msg    - Message to print with the entry
 *   ipaddr - The IP address to dump
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void neighbor_dumpipaddr(FAR const char *msg,
                         const net_ipv6addr_t ipaddr)
{
  ninfo("%s: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        msg,
        ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]),
        ntohs(ipaddr[3]), ntohs(ipaddr[4]), ntohs(ipaddr[5]),
        ntohs(ipaddr[6]), ntohs(ipaddr[7]));
}

#endif /* CONFIG_DEBUG_NET_INFO */
