/****************************************************************************
 * net/inet/inet_globals.c
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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#ifdef CONFIG_NET_IPv4
/* Increasing number used for the IP ID field. */

uint16_t g_ipid;
#endif /* CONFIG_NET_IPv4 */

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

#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER)
const net_ipv6addr_t g_ipv6_allrouters =  /* All link local routers */
{
  HTONS(0xff02),
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  HTONS(0x0002)
};

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
/* Link-Local Address: Link-local addresses have "1111 1110 10" for the
 * first ten bits followed by 54 zeroes and then the 64 bit interface
 * identifier (typically derived from the data link layer address).
 */

const net_ipv6addr_t g_ipv6_llnetmask =   /* Netmask for local link address */
{
  0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0x0000, 0x0000, 0x0000
};
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

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
#endif /* CONFIG_NET_ICMPv6_AUTOCONF || CONFIG_NET_ICMPv6_ROUTER */
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
