/****************************************************************************
 * net/icmpv6/icmpv6_send.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_ICMPv6) && defined(CONFIG_NET_ICMPv6_PING)

#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

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
 * Name: icmpv6_send
 *
 * Description:
 *   Setup to send an ICMPv6 packet
 *
 * Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void icmpv6_send(FAR struct net_driver_s *dev, FAR net_ipv6addr_t *destaddr)
{
  FAR struct icmpv6_iphdr_s *picmpv6 = ICMPv6BUF;

  if (dev->d_sndlen > 0)
    {
      /* The total length to send is the size of the application data plus
       * the IP and ICMPv6 headers (and, eventually, the Ethernet header)
       */

      dev->d_len = dev->d_sndlen + IPICMPv6_HDRLEN;

      /* The total size of the data (for ICMPv6 checksum calculation) includes
       * the size of the ICMPv6 header
       */

      dev->d_sndlen += ICMPv6_HDRLEN;

      /* Initialize the IP header.  Note that for IPv6, the IP length field
       * does not include the IPv6 IP header length.
       */

      picmpv6->vtc         = 0x60;
      picmpv6->tcf         = 0x00;
      picmpv6->flow        = 0x00;
      picmpv6->len[0]      = (dev->d_sndlen >> 8);
      picmpv6->len[1]      = (dev->d_sndlen & 0xff);
      picmpv6->nexthdr     = IP_PROTO_ICMPv6;
      picmpv6->hoplimit    = IP_TTL;

      net_ipv6addr_copy(picmpv6->srcipaddr, &dev->d_ipaddr);
      net_ipv6addr_copy(picmpv6->destipaddr, destaddr);

      /* Calculate the ICMPv6 checksum. */

      picmpv6->icmpv6chksum  = 0;
      picmpv6->icmpv6chksum  = ~(icmpv6_chksum(dev, dev->d_sndlen));
      if (picmpv6->icmpv6chksum == 0)
        {
          picmpv6->icmpv6chksum = 0xffff;
        }

      nllvdbg("Outgoing ICMPv6 packet length: %d (%d)\n",
              dev->d_len, (picmpv6->len[0] << 8) | picmpv6->len[1]);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.icmpv6.sent++;
      g_netstats.ip.sent++;
#endif
    }
}

#endif /* CONFIG_NET && CONFIG_NET_ICMPv6 && CONFIG_NET_ICMPv6_PING */
