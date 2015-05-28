/****************************************************************************
 * net/icmp/icmp_input.c
 * Handling incoming ICMP input
 *
 *   Copyright (C) 2007-2009, 2012, 2014-2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <net/if.h>
#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "icmp/icmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_ICMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPBUF ((struct icmp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_input
 *
 * Description:
 *   Handle incoming ICMP input
 *
 * Parameters:
 *   dev - The device driver structure containing the received ICMP
 *         packet
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void icmp_input(FAR struct net_driver_s *dev)
{
  FAR struct icmp_iphdr_s *picmp = ICMPBUF;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.recv++;
#endif

  /* ICMP echo (i.e., ping) processing. This is simple, we only change the
   * ICMP type from ECHO to ECHO_REPLY and adjust the ICMP checksum before
   * we return the packet.
   */

  if (picmp->type == ICMP_ECHO_REQUEST)
    {
      /* If we are configured to use ping IP address assignment, we use
       * the destination IP address of this ping packet and assign it to
       * ourself.
       */

#ifdef CONFIG_NET_PINGADDRCONF
      if (dev->d_ipaddr == 0)
        {
          dev->d_ipaddr = picmp->destipaddr;
        }
#endif

      /* Change the ICMP type */

      picmp->type = ICMP_ECHO_REPLY;

      /* Swap IP addresses. */

      net_ipv4addr_hdrcopy(picmp->destipaddr, picmp->srcipaddr);
      net_ipv4addr_hdrcopy(picmp->srcipaddr, &dev->d_ipaddr);

      /* Recalculate the ICMP checksum */

#if 0
      /* The slow way... sum over the ICMP message */

      picmp->icmpchksum = 0;
      picmp->icmpchksum = ~icmp_chksum(dev, (((uint16_t)picmp->len[0] << 8) | (uint16_t)picmp->len[1]) - IPv4_HDRLEN);
      if (picmp->icmpchksum == 0)
        {
          picmp->icmpchksum = 0xffff;
        }
#else
      /* The quick way -- Since only the type has changed, just adjust the
       * checksum for the change of type
       */

      if (picmp->icmpchksum >= HTONS(0xffff - (ICMP_ECHO_REQUEST << 8)))
        {
          picmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8) + 1;
        }
      else
        {
          picmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8);
        }
#endif

      nllvdbg("Outgoing ICMP packet length: %d (%d)\n",
              dev->d_len, (picmp->len[0] << 8) | picmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.icmp.sent++;
      g_netstats.ipv4.sent++;
#endif
    }

  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

#ifdef CONFIG_NET_ICMP_PING
  else if (picmp->type == ICMP_ECHO_REPLY && dev->d_conncb)
    {
      (void)devif_conn_event(dev, picmp, ICMP_ECHOREPLY, dev->d_conncb);
    }
#endif

  /* Otherwise the ICMP input was not processed */

  else
    {
      nlldbg("Unknown ICMP cmd: %d\n", picmp->type);
      goto typeerr;
    }

  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.typeerr++;
  g_netstats.icmp.drop++;
#endif
  dev->d_len = 0;
}

#endif /* CONFIG_NET_ICMP */
#endif /* CONFIG_NET */
