/****************************************************************************
 * net/uip/uip-icmpinput.c
 * Handling incoming ICMP/ICMP6 input
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <debug.h>

#include <net/if.h>
#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

#ifdef CONFIG_NET_ICMP

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

#ifdef CONFIG_NET_ICMP_PING
struct uip_callback_s *g_echocallback = NULL;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_icmpinput
 *
 * Description:
 *   Handle incoming ICMP/ICMP6 input
 *
 * Parameters:
 *   dev - The device driver structure containing the received ICMP/ICMP6
 *         packet
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_icmpinput(struct uip_driver_s *dev)
{
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.recv++;
#endif

#ifndef CONFIG_NET_IPv6
  /* ICMPv4 processing code follows. */

  /* ICMP echo (i.e., ping) processing. This is simple, we only change the
   * ICMP type from ECHO to ECHO_REPLY and adjust the ICMP checksum before
   * we return the packet.
   */

  if (ICMPBUF->type == ICMP_ECHO_REQUEST)
    {
      /* If we are configured to use ping IP address assignment, we use
       * the destination IP address of this ping packet and assign it to
       * ourself.
       */

#ifdef CONFIG_NET_PINGADDRCONF
      if (dev->d_ipaddr == 0)
        {
          dev->d_ipaddr = ICMPBUF->destipaddr;
        }
#endif

      ICMPBUF->type = ICMP_ECHO_REPLY;

      if (ICMPBUF->icmpchksum >= HTONS(0xffff - (ICMP_ECHO_REPLY << 8)))
        {
          ICMPBUF->icmpchksum += HTONS(ICMP_ECHO_REPLY << 8) + 1;
        }
      else
        {
          ICMPBUF->icmpchksum += HTONS(ICMP_ECHO_REPLY << 8);
        }

      /* Swap IP addresses. */

      uiphdr_ipaddr_copy(ICMPBUF->destipaddr, ICMPBUF->srcipaddr);
      uiphdr_ipaddr_copy(ICMPBUF->srcipaddr, &dev->d_ipaddr);

      nvdbg("Outgoing ICMP packet length: %d (%d)\n",
            dev->d_len, (ICMPBUF->len[0] << 8) | ICMPBUF->len[1]);

#ifdef CONFIG_NET_STATISTICS
      uip_stat.icmp.sent++;
      uip_stat.ip.sent++;
#endif
    }

  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

#ifdef CONFIG_NET_ICMP_PING
  else if (ICMPBUF->type == ICMP_ECHO_REPLY && g_echocallback)
    {
      (void)uip_callbackexecute(dev, ICMPBUF, UIP_ECHOREPLY, g_echocallback);
    }
#endif

  /* Otherwise the ICMP input was not processed */

  else
    {
      ndbg("Unknown ICMP cmd: %d\n", ICMPBUF->type);
      goto typeerr;
    }

  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.typeerr++;
  uip_stat.icmp.drop++;
#endif
  dev->d_len = 0;

#else /* !CONFIG_NET_IPv6 */

  /* If we get a neighbor solicitation for our address we should send
   * a neighbor advertisement message back.
   */

  if (ICMPBUF->type == ICMP6_NEIGHBOR_SOLICITATION)
    {
      if (uip_ipaddr_cmp(ICMPBUF->icmp6data, dev->d_ipaddr))
        {
          if (ICMPBUF->options[0] == ICMP6_OPTION_SOURCE_LINK_ADDRESS)
            {
              /* Save the sender's address in our neighbor list. */

              uiphdr_neighbor_add(ICMPBUF->srcipaddr, &(ICMPBUF->options[2]));
            }

          /* We should now send a neighbor advertisement back to where the
           * neighbor solicication came from.
           */

          ICMPBUF->type = ICMP6_NEIGHBOR_ADVERTISEMENT;
          ICMPBUF->flags = ICMP6_FLAG_S; /* Solicited flag. */

          ICMPBUF->reserved1 = ICMPBUF->reserved2 = ICMPBUF->reserved3 = 0;

          uiphdr_ipaddr_copy(ICMPBUF->destipaddr, ICMPBUF->srcipaddr);
          uiphdr_ipaddr_copy(ICMPBUF->srcipaddr, &dev->d_ipaddr);
          ICMPBUF->options[0] = ICMP6_OPTION_TARGET_LINK_ADDRESS;
          ICMPBUF->options[1] = 1;  /* Options length, 1 = 8 bytes. */
          memcpy(&(ICMPBUF->options[2]), &dev->d_mac, IFHWADDRLEN);
          ICMPBUF->icmpchksum = 0;
          ICMPBUF->icmpchksum = ~uip_icmp6chksum(dev);
        }
      else
        {
          goto drop;
        }
    }
  else if (ICMPBUF->type == ICMP6_ECHO_REQUEST)
    {
      /* ICMP echo (i.e., ping) processing. This is simple, we only
       * change the ICMP type from ECHO to ECHO_REPLY and update the
       * ICMP checksum before we return the packet.
       */

      ICMPBUF->type = ICMP6_ECHO_REPLY;

      uiphdr_ipaddr_copy(ICMPBUF->destipaddr, ICMPBUF->srcipaddr);
      uiphdr_ipaddr_copy(ICMPBUF->srcipaddr, &dev->d_ipaddr);
      ICMPBUF->icmpchksum = 0;
      ICMPBUF->icmpchksum = ~uip_icmp6chksum(dev);
    }

  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

#ifdef CONFIG_NET_ICMP_PING
  else if (ICMPBUF->type == ICMP6_ECHO_REPLY && g_echocallback)
    {
      (void)uip_callbackexecute(dev, ICMPBUF, UIP_ECHOREPLY, g_echocallback);
    }
#endif

  else
    {
      ndbg("Unknown ICMP6 cmd: %d\n", ICMPBUF->type);
      goto typeerr;
    }

  nvdbg("Outgoing ICMP6 packet length: %d (%d)\n",
        dev->d_len, (ICMPBUF->len[0] << 8) | ICMPBUF->len[1]);

#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.sent++;
  uip_stat.ip.sent++;
#endif
  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.typeerr++;
#endif

drop:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.drop++;
#endif
  dev->d_len = 0;

#endif /* !CONFIG_NET_IPv6 */
}

#endif /* CONFIG_NET_ICMP */
#endif /* CONFIG_NET */
