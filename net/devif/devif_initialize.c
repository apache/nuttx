/****************************************************************************
 * net/devif/devif_initialize.c
 *
 *   Copyright (C) 2007-2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>

#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* IP/TCP/UDP/ICMP statistics for all network interfaces */

#ifdef CONFIG_NET_STATISTICS
struct net_stats_s g_netstats;
#endif

/* Increasing number used for the IP ID field. */

uint16_t g_ipid;

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_TCP_REASSEMBLY)
/* Reassembly timer (units: deci-seconds) */

uint8_t g_reassembly_timer;
#endif

#ifdef CONFIG_NET_IPv6

const net_ipv6addr_t g_ipv6_alloneaddr =  /* An address of all ones */
{
  0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff
};

const net_ipv6addr_t g_ipv6_allzeroaddr = /* An address of all zeroes */
{
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER)
/* IPv6 Multi-cast IP addresses */

const net_ipv6addr_t g_ipv6_allnodes =    /* All link local nodes */
{
  HTONS(0xff02),
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  HTONS(0x0001)
};

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
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_initialize
 *
 * Description:
 *   Perform initialization of the network device interface layer
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void devif_initialize(void)
{
  /* Initialize callback support */

  devif_callback_init();
}
#endif /* CONFIG_NET */
