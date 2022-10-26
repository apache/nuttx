/****************************************************************************
 * net/arp/arp_arpin.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_arpin
 *
 * Description:
 *   This function should be called by the Ethernet device driver when an
 *   ARP packet has been received.   The function will act differently
 *   depending on the ARP packet type: if it is a reply for a request
 *   that we previously sent out, the ARP cache will be filled in with
 *   the values from the ARP reply.  If the incoming ARP packet is an ARP
 *   request for our IP address, an ARP reply packet is created and put
 *   into the d_buf buffer.
 *
 *   On entry, this function expects that an ARP packet with a prepended
 *   Ethernet header is present in the d_buf buffer and that the length of
 *   the packet is set in the d_len field.
 *
 *   When the function returns, the value of the field d_len indicates
 *   whether the device driver should send out the ARP reply packet or not.
 *   If d_len is zero, no packet should be sent; If d_len is non-zero, it
 *   contains the length of the outbound packet that is present in the
 *   d_buf buffer.
 *
 ****************************************************************************/

void arp_arpin(FAR struct net_driver_s *dev)
{
  FAR struct arp_hdr_s *arp = ARPBUF;
  in_addr_t ipaddr;

  if (dev->d_len < (sizeof(struct arp_hdr_s) + ETH_HDRLEN))
    {
      nerr("ERROR: Packet Too small\n");
      dev->d_len = 0;
      return;
    }

  dev->d_len = 0;

  ipaddr = net_ip4addr_conv32(arp->ah_dipaddr);
  switch (arp->ah_opcode)
    {
      case HTONS(ARP_REQUEST):
        ninfo("ARP request for IP %04lx\n", (unsigned long)ipaddr);

        /* ARP request. If it asked for our address, we send out a reply. */

        if (net_ipv4addr_cmp(ipaddr, dev->d_ipaddr))
          {
            FAR struct eth_hdr_s *eth = ETHBUF;

            /* First, we register the one who made the request in our ARP
             * table, since it is likely that we will do more communication
             * with this host in the future.
             */

            arp_hdr_update(dev, arp->ah_sipaddr, arp->ah_shwaddr);

            arp->ah_opcode = HTONS(ARP_REPLY);
            memcpy(arp->ah_dhwaddr, arp->ah_shwaddr, ETHER_ADDR_LEN);
            memcpy(arp->ah_shwaddr, dev->d_mac.ether.ether_addr_octet,
                   ETHER_ADDR_LEN);
            memcpy(eth->src, dev->d_mac.ether.ether_addr_octet,
                   ETHER_ADDR_LEN);
            memcpy(eth->dest, arp->ah_dhwaddr, ETHER_ADDR_LEN);

            arp->ah_dipaddr[0] = arp->ah_sipaddr[0];
            arp->ah_dipaddr[1] = arp->ah_sipaddr[1];
            net_ipv4addr_hdrcopy(arp->ah_sipaddr, &dev->d_ipaddr);
            arp_dump(arp);

            eth->type           = HTONS(ETHTYPE_ARP);
            dev->d_len          = sizeof(struct arp_hdr_s) + ETH_HDRLEN;
          }
        break;

      case HTONS(ARP_REPLY):
        ninfo("ARP reply for IP %04lx\n", (unsigned long)ipaddr);

        /* ARP reply. We insert or update the ARP table if it was meant
         * for us.
         */

        if (net_ipv4addr_cmp(ipaddr, dev->d_ipaddr))
          {
            /* Yes... Insert the address mapping in the ARP table */

            arp_hdr_update(dev, arp->ah_sipaddr, arp->ah_shwaddr);

            /* Then notify any logic waiting for the ARP result */

            arp_notify(net_ip4addr_conv32(arp->ah_sipaddr));
          }
        break;
    }
}

#endif /* CONFIG_NET_ARP */
