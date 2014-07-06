/****************************************************************************
 * net/ipv6/ipv6.h
 * Header file for database of link-local neighbors, used by IPv6 code and
 * to be used by future ARP code.
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * A direct leverage of logic from uIP which also has b BSD style license
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

#ifndef __NET_IPV6_IPV6_H
#define __NET_IPV6_IPV6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <net/ethernet.h>

#include <nuttx/net/ip.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct net_neighbor_addr_s
{
#if CONFIG_NET_IPV6_NEIGHBOR_ADDRTYPE
  CONFIG_NET_IPV6_NEIGHBOR_ADDRTYPE addr;
#else
  struct ether_addr addr;
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void net_neighbor_init(void);
void net_neighbor_add(net_ipaddr_t ipaddr, struct net_neighbor_addr_s *addr);
void net_neighbor_update(net_ipaddr_t ipaddr);
struct net_neighbor_addr_s *net_neighbor_lookup(net_ipaddr_t ipaddr);
void net_neighbor_periodic(void);

#endif /* __NET_IPV6_IPV6_H */
