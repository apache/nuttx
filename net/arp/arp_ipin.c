/****************************************************************************
 * net/arp/arp_ipin.c
 *
 *   Copyright (C) 2007-2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
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

#include <netinet/in.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_IPIN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_ipin
 *
 * Description:
 *   The arp_ipin() function should be called by Ethernet device drivers
 *   whenever an IP packet arrives from the network.  The function will
 *   check if the address is in the ARP cache, and if so the ARP cache entry
 *   will be refreshed. If no ARP cache entry was found, a new one is
 *   created.
 *
 *   This function expects that an IP packet with an Ethernet header is
 *   present in the d_buf buffer and that the length of the packet is in the
 *   d_len field.
 *
 ****************************************************************************/

void arp_ipin(FAR struct net_driver_s *dev)
{
  in_addr_t srcipaddr;

  /* ARP support is only built if the Ethernet link layer is supported.
   * Continue and send the ARP request only if this device uses the
   * Ethernet link layer protocol.
   */

  if (dev->d_lltype != NET_LL_ETHERNET &&
      dev->d_lltype != NET_LL_IEEE80211)
    {
      return;
    }

  /* Only insert/update an entry if the source IP address of the incoming IP
   * packet comes from a host on the local network.
   */

  srcipaddr = net_ip4addr_conv32(ARPIPBUF->eh_srcipaddr);
  if (net_ipv4addr_maskcmp(srcipaddr, dev->d_ipaddr, dev->d_netmask))
    {
      arp_hdr_update(dev, ARPIPBUF->eh_srcipaddr, ETHBUF->src);
    }
}

#endif /* CONFIG_NET_ARP_IPIN */
