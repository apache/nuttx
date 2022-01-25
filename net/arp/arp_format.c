/****************************************************************************
 * net/arp/arp_format.c
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

#include <string.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#include "arp/arp.h"

#ifdef CONFIG_NET_ARP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF ((FAR struct eth_hdr_s *)&dev->d_buf[0])
#define ARPBUF ((FAR struct arp_hdr_s *)&dev->d_buf[ETH_HDRLEN])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_format
 *
 * Description:
 *   Format an ARP packet.
 *
 * Input Parameters:
 *   dev    - Device structure
 *   ipaddr - Target IP address (32-bit)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arp_format(FAR struct net_driver_s *dev, in_addr_t ipaddr)
{
  FAR struct arp_hdr_s *arp = ARPBUF;
  FAR struct eth_hdr_s *eth = ETHBUF;

  /* Construct the ARP packet.  Creating both the Ethernet and ARP headers */

  memset(eth->dest, 0xff, ETHER_ADDR_LEN);
  memset(arp->ah_dhwaddr, 0x00, ETHER_ADDR_LEN);
  memcpy(eth->src, dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);
  memcpy(arp->ah_shwaddr, dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);

  net_ipv4addr_hdrcopy(arp->ah_dipaddr, &ipaddr);
  net_ipv4addr_hdrcopy(arp->ah_sipaddr, &dev->d_ipaddr);

  arp->ah_opcode   = HTONS(ARP_REQUEST);
  arp->ah_hwtype   = HTONS(ARP_HWTYPE_ETH);
  arp->ah_protocol = HTONS(ETHTYPE_IP);
  arp->ah_hwlen    = ETHER_ADDR_LEN;
  arp->ah_protolen = 4;

  eth->type        = HTONS(ETHTYPE_ARP);
  dev->d_len       = sizeof(struct arp_hdr_s) + ETH_HDRLEN;
}

#endif /* CONFIG_NET_ARP */
