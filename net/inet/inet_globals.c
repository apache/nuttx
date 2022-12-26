/****************************************************************************
 * net/inet/inet_globals.c
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

#include <arpa/inet.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_ETHERNET
#  include <net/ethernet.h>
#endif

#include "inet/inet.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6

/* Unspecified address (all zero).  See RFC 4291 (replaces 3513) */

const net_ipv6addr_t g_ipv6_unspecaddr =  /* An address of all zeroes */
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

/* IPv6 Multi-cast IP addresses.  See RFC 2375 */

const net_ipv6addr_t g_ipv6_allnodes =    /* All link local nodes */
{
  HTONS(0xff02),
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  HTONS(0x0001)
};

#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER) || \
    defined(CONFIG_NET_MLD)
const net_ipv6addr_t g_ipv6_allrouters =  /* All link local routers */
{
  HTONS(0xff02),
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  HTONS(0x0002)
};

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
/* Link-Local Address: Link-local addresses have "1111 1110 10" for the
 * first ten bits followed by 54 zeroes and then the 64 bit interface
 * identifier (typically derived from the link layer MAC address).
 */

const net_ipv6addr_t g_ipv6_llnetmask =   /* Netmask for local link address */
{
  0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0x0000, 0x0000, 0x0000
};
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_MLD
/* Version 2 Multicast Listener Reports are sent with an IP destination
 * address of FF02:0:0:0:0:0:0:16 on which all MLDv2-capable multicast
 * routers listen (RFC 3810)
 */

const net_ipv6addr_t g_ipv6_allmldv2routers =  /* All MLDv2 link local routers */
{
  HTONS(0xff02),
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  HTONS(0x0016)
};
#endif

#ifdef CONFIG_NET_ETHERNET

/* IPv6 Multi-cast Ethernet addresses.  Formed from the 16-bit prefix:
 *
 *   0x33:0x33:xx:xx:xx:xx:
 *
 * and the last 32-bits of the IPv6 IP address
 */

const struct ether_addr g_ipv6_ethallnodes =     /* All link local nodes */
{
  { 0x33, 0x33, 0x00, 0x00, 0x00, 0x01 }
};

const struct ether_addr g_ipv6_ethallrouters =   /* All link local routers */
{
  { 0x33, 0x33, 0x00, 0x00, 0x00, 0x02 }
};

#endif /* CONFIG_NET_ETHERNET */
#endif /* CONFIG_NET_ICMPv6_AUTOCONF || CONFIG_NET_ICMPv6_ROUTER || CONFIG_NET_MLD */
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
