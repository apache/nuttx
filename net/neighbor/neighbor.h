/****************************************************************************
 * net/neighbor/neighbor.h
 * Header file for database of link-local neighbors, used by IPv6 code and
 * to be used by future ARP code.
 *
 *   Copyright (C) 2007-2009, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * A leverage of logic from uIP which also has a BSD style license
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2006, Swedish Institute of Computer Science.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#ifndef __NET_NEIGHBOR_NEIGHBOR_H
#define __NET_NEIGHBOR_NEIGHBOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <net/ethernet.h>

#include <nuttx/clock.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_IPv6_NCONF_ENTRIES
#  define CONFIG_NET_IPv6_NCONF_ENTRIES 8
#endif

#define NEIGHBOR_MAXTIME 128

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes the link layer address */

struct neighbor_addr_s
{
  uint8_t                    na_lltype;
  uint8_t                    na_llsize;

  union
  {
#ifdef CONFIG_NET_ETHERNET
    struct ether_addr        na_ethernet;
#endif
#ifdef CONFIG_NET_6LOWPAN
    struct netdev_maxaddr_s  na_sixlowpan;
#endif
  } u;
};

/* This structure describes on entry in the neighbor table.  This is intended
 * for internal use within the Neighbor implementation.
 */

struct neighbor_entry
{
  net_ipv6addr_t         ne_ipaddr;  /* IPv6 address of the Neighbor */
  struct neighbor_addr_s ne_addr;    /* Link layer address of the Neighbor */
  uint8_t                ne_time;    /* For aging, units of half seconds */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the Neighbor table.  The network should be locked when accessing
 * this table.
 */

extern struct neighbor_entry g_neighbors[CONFIG_NET_IPv6_NCONF_ENTRIES];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s; /* Forward reference */

/****************************************************************************
 * Name: neighbor_initialize
 *
 * Description:
 *   Initialize Neighbor table data structures.  This function is called
 *   prior to platform-specific driver initialization so that the networking
 *   subsystem is prepared to deal with network driver initialization
 *   actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_initialize(void);

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

FAR struct neighbor_entry *neighbor_findentry(const net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name: neighbor_add
 *
 * Description:
 *   Add the new address association to the Neighbor Table (if it is not
 *   already there).
 *
 * Input Parameters:
 *   dev    - Driver instance associated with the MAC
 *   ipaddr - The IPv6 address of the mapping.
 *   addr   - The link layer address of the mapping
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_add(FAR struct net_driver_s *dev, FAR net_ipv6addr_t ipaddr,
                  FAR uint8_t *addr);

/****************************************************************************
 * Name:  neighbor_lookup
 *
 * Description:
 *   Find an entry in the Neighbor Table and return its link layer address.
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address to use in the lookup;
 *
 * Returned Value:
 *   A read-only reference to the link layer address in the Neighbor Table is
 *   returned on success.  NULL is returned if there is no matching entry in
 *   the Neighbor Table.
 *
 ****************************************************************************/

FAR const struct neighbor_addr_s *neighbor_lookup(const net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name: neighbor_update
 *
 * Description:
 *   Reset time on the Neighbor Table entry associated with the IPv6 address.
 *   This makes the associated entry the most recently used and not a
 *   candidate for removal.
 *
 * Input Parameters:
 *   ipaddr - The IPv6 address of the entry to be updated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_update(const net_ipv6addr_t ipaddr);

/****************************************************************************
 * Name: neighbor_periodic
 *
 * Description:
 *   Called from the timer poll logic in order to perform agin operations on
 *   entries in the Neighbor Table
 *
 * Input Parameters:
 *   hsec - Elapsed time in half seconds since the last check
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void neighbor_periodic(int hsec);

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

#ifdef CONFIG_DEBUG_NET_INFO
void neighbor_dumpentry(FAR const char *msg,
                        FAR struct neighbor_entry *neighbor);
#else
#  define neighbor_dumpentry(msg,neighbor)
#endif

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

#ifdef CONFIG_DEBUG_NET_INFO
void neighbor_dumpipaddr(FAR const char *msg,
                         const net_ipv6addr_t ipaddr);
#else
#  define neighbor_dumpipaddr(msg,ipaddr)
#endif

#endif /* CONFIG_NET_IPv6 */
#endif /* __NET_NEIGHBOR_NEIGHBOR_H */

