/****************************************************************************
 * net/uip/uip-icmsend.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING)

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ICMPBUF ((struct uip_icmpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

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
 * Name: uip_icmpsend
 *
 * Description:
 *   Setup to send an ICMP packet
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

void uip_icmpsend(struct uip_driver_s *dev, uip_ipaddr_t *destaddr)
{
  if (dev->d_sndlen > 0)
    {
      /* The total lenth to send is the size of the application data plus
       * the IP and ICMP headers (and, eventually, the ethernet header)
       */

      dev->d_len = dev->d_sndlen + UIP_IPICMPH_LEN;

      /* Initialize the IP header.  Note that for IPv6, the IP length field
       * does not include the IPv6 IP header length.
       */

#ifdef CONFIG_NET_IPv6

      ICMPBUF->vtc         = 0x60;
      ICMPBUF->tcf         = 0x00;
      ICMPBUF->flow        = 0x00;
      ICMPBUF->len[0]      = (dev->d_sndlen >> 8);
      ICMPBUF->len[1]      = (dev->d_sndlen & 0xff);
      ICMPBUF->nexthdr     = UIP_PROTO_ICMP;
      ICMPBUF->hoplimit    = UIP_TTL;

      uip_ipaddr_copy(ICMPBUF->srcipaddr, &dev->d_ipaddr);
      uip_ipaddr_copy(ICMPBUF->destipaddr, destaddr);

#else /* CONFIG_NET_IPv6 */

      ICMPBUF->vhl         = 0x45;
      ICMPBUF->tos         = 0;
      ICMPBUF->len[0]      = (dev->d_len >> 8);
      ICMPBUF->len[1]      = (dev->d_len & 0xff);
      ++g_ipid;
      ICMPBUF->ipid[0]     = g_ipid >> 8;
      ICMPBUF->ipid[1]     = g_ipid & 0xff;
      ICMPBUF->ipoffset[0] = 0;
      ICMPBUF->ipoffset[1] = 0;
      ICMPBUF->ttl         = UIP_TTL;
      ICMPBUF->proto       = UIP_PROTO_ICMP;

      /* Calculate IP checksum. */

      ICMPBUF->ipchksum    = 0;
      ICMPBUF->ipchksum    = ~(uip_ipchksum(dev));

      uiphdr_ipaddr_copy(ICMPBUF->srcipaddr, &dev->d_ipaddr);
      uiphdr_ipaddr_copy(ICMPBUF->destipaddr, destaddr);

#endif /* CONFIG_NET_IPv6 */

      /* Calculate the ICMP checksum. */

      ICMPBUF->icmpchksum   = 0;
      ICMPBUF->icmpchksum   = ~(uip_icmpchksum(dev));
      if (ICMPBUF->icmpchksum == 0)
        {
          ICMPBUF->icmpchksum = 0xffff;
        }

      nvdbg("Outgoing ICMP packet length: %d (%d)\n",
            dev->d_len, (ICMPBUF->len[0] << 8) | ICMPBUF->len[1]);

#ifdef CONFIG_NET_STATISTICS
      uip_stat.icmp.sent++;
      uip_stat.ip.sent++;
#endif
    }
}

#endif /* CONFIG_NET && CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING */
