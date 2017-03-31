/****************************************************************************
 * net/sixlowpan/sixlowpan_input.c
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

#include <errno.h>

#include "nuttx/net/netdev.h"
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_isbroadcast
 *
 * Description:
 *   Return the address length associated with a 2-bit address mode
 *
 * Input parameters:
 *   addrmode - The address mode
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static bool sixlowpan_isbroadcast(uint8_t mode, FAR uint8_t *addr)
{
  int i = ((mode == FRAME802154_SHORTADDRMODE) ? 2 : 8);

  while (i-- > 0)
    {
      if (addr[i] != 0xff)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: sixlowpan_set_pktattrs
 *
 * Description:
 *   Setup some packet buffer attributes
 *
 * Input Parameters:
 *   ieee - Pointer to IEEE802.15.4 MAC driver structure.
 *   ipv6 - Pointer to the IPv6 header to "compress"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_set_pktattrs(FAR struct ieee802154_driver_s *ieee,
                                   FAR const struct ipv6_hdr_s *ipv6)
{
  int attr = 0;

  /* Set protocol in NETWORK_ID */

  g_pktattrs[PACKETBUF_ATTR_NETWORK_ID] = ipv6->proto;

  /* Assign values to the channel attribute (port or type + code) */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      FAR struct udp_hdr_s *udp = &((FAR struct ipv6udp_hdr_s *)ipv6)->udp;

      attr = udp->srcport;
      if (udp->destport < attr)
        {
          attr = udp->destport;
        }
    }
  else if (ipv6->proto == IP_PROTO_TCP)
    {
      FAR struct tcp_hdr_s *tcp = &((FAR struct ipv6tcp_hdr_s *)ipv6)->tcp;

      attr = tcp->srcport;
      if (tcp->destport < attr)
        {
          attr = tcp->destport;
        }
    }
  else if (ipv6->proto == IP_PROTO_ICMP6)
    {
      FAR struct icmpv6_iphdr_s *icmp = &((FAR struct ipv6icmp_hdr_s *)ipv6)->icmp;

      attr = icmp->type << 8 | icmp->code;
    }

  g_pktattrs[PACKETBUF_ATTR_CHANNEL] = attr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_input
 *
 * Description:
 *   Process an incoming IP packet.
 *
 *   This function is called when the device driver has received a 6loWPAN
 *   packet from the network. The packet from the device driver must be
 *   present in the d_buf buffer, and the length of the packet should be
 *   placed in the d_len field.
 *
 *   When the function returns, there may be an outbound packet placed
 *   in the d_buf packet buffer. If so, the d_len field is set to
 *   the length of the packet. If no packet is to be sent out, the
 *   d_len field is set to 0.
 *
 * Input Parameters:
 *   dev - The IEEE802.15.4 MAC network driver interface.
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 ****************************************************************************/

int sixlowpan_input(FAR struct net_driver_s *dev)
{
  /* REVISIT: To be provided */
  return -ENOSYS;
}

#endif /* CONFIG_NET_6LOWPAN */
