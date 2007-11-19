/****************************************************************************
 * net/uip/uip-udpsend.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define UDPBUF  ((struct uip_udpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

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
 * Name: uip_udpsend
 *
 * Description:
 *   Setup to send a UDP packet
 *
 * Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The UDP "connection" structure holding port information
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_udpsend(struct uip_driver_s *dev, struct uip_udp_conn *conn)
{
  if (dev->d_sndlen == 0)
    {
      /* The total lenth to send is the size of the application data plus
       * the IP and UDP headers (and, eventually, the ethernet header)
       */

      dev->d_len = dev->d_sndlen + UIP_IPUDPH_LEN;

      /* Initialize the IP header.  Note that for IPv6, the IP length field
       * does not include the IPv6 IP header length.
       */

#ifdef CONFIG_NET_IPv6

      UDPBUF->vtc         = 0x60;
      UDPBUF->tcf         = 0x00;
      UDPBUF->flow        = 0x00;
      UDPBUF->len[0]      = (dev->d_sndlen >> 8);
      UDPBUF->len[1]      = (dev->d_sndlen & 0xff);
      UDPBUF->nexthdr     = UIP_PROTO_UDP;
      UDPBUF->hoplimit    = conn->ttl;

      uip_ipaddr_copy(UDPBUF->srcipaddr, &dev->d_ipaddr);
      uip_ipaddr_copy(UDPBUF->destipaddr, &conn->ripaddr);

#else /* CONFIG_NET_IPv6 */

      UDPBUF->vhl         = 0x45;
      UDPBUF->tos         = 0;
      UDPBUF->len[0]      = (dev->d_len >> 8);
      UDPBUF->len[1]      = (dev->d_len & 0xff);
      ++g_ipid;
      UDPBUF->ipid[0]     = g_ipid >> 8;
      UDPBUF->ipid[1]     = g_ipid & 0xff;
      UDPBUF->ipoffset[0] = 0;
      UDPBUF->ipoffset[1] = 0;
      UDPBUF->ttl          = conn->ttl;
      UDPBUF->proto        = UIP_PROTO_UDP;

      /* Calculate IP checksum. */

      UDPBUF->ipchksum    = 0;
      UDPBUF->ipchksum    = ~(uip_ipchksum(dev));

      uiphdr_ipaddr_copy(UDPBUF->srcipaddr, &dev->d_ipaddr);
      uiphdr_ipaddr_copy(UDPBUF->destipaddr, &conn->ripaddr);

#endif /* CONFIG_NET_IPv6 */

      /* Initialize the UDP header */

      UDPBUF->srcport      = conn->lport;
      UDPBUF->destport     = conn->rport;
      UDPBUF->udplen       = HTONS(dev->d_sndlen + UIP_UDPH_LEN);

#ifdef CONFIG_NET_UDP_CHECKSUMS
      /* Calculate UDP checksum. */

      UDPBUF->udpchksum = ~(uip_udpchksum(dev));
      if (UDPBUF->udpchksum == 0)
        {
          UDPBUF->udpchksum = 0xffff;
        }
#else
      UDPBUF->udpchksum    = 0;
#endif

      vdbg("Outgoing UDP packet length: %d (%d)\n",
           dev->d_len, (UDPBUF->len[0] << 8) | UDPBUF->len[1]);

#ifdef CONFIG_NET_STATISTICS
      uip_stat.udp.sent++;
      uip_stat.ip.sent++;
#endif
    }
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
