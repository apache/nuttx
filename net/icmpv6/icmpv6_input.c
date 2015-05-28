/****************************************************************************
 * net/icmpv6/icmpv6_input.c
 * Handling incoming ICMPv6 input
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/icmpv6.h>

#include "devif/devif.h"
#include "neighbor/neighbor.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF    ((struct eth_hdr_s *)&dev->d_buf[0])
#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define ICMPv6SOLICIT \
  ((struct icmpv6_neighbor_solicit_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6ADVERTISE \
  ((struct icmpv6_neighbor_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])
#define ICMPv6RADVERTISE \
  ((struct icmpv6_router_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_input
 *
 * Description:
 *   Handle incoming ICMPv6 input
 *
 * Parameters:
 *   dev - The device driver structure containing the received ICMPv6
 *         packet
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_iphdr_s *icmp = ICMPv6BUF;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.recv++;
#endif

  /* Handle the ICMPv6 message by its type */

  switch (icmp->type)
    {
    /* If we get a neighbor solicitation for our address we should send
     * a neighbor advertisement message back.
     */

    case ICMPv6_NEIGHBOR_SOLICIT:
      {
        FAR struct icmpv6_neighbor_solicit_s *sol;

        /* Check if we are the target of the solicitation */

        sol = ICMPv6SOLICIT;
        if (net_ipv6addr_cmp(sol->tgtaddr, dev->d_ipv6addr))
          {
            /* Yes..  Send a neighbor advertisement back to where the neighbor
             * solicitation came from.
             */

            icmpv6_advertise(dev, icmp->srcipaddr);

            /* All statistics have been updated.  Nothing to do but exit. */

            return;
          }
        else
          {
            goto icmpv6_drop_packet;
          }
      }
      break;

    /* Check if we received a Neighbor Advertisement */

    case ICMPv6_NEIGHBOR_ADVERTISE:
      {
        FAR struct icmpv6_neighbor_advertise_s *adv;

        /* If the IPv6 destination address matches our address, and if so,
         * add the neighbor address mapping to the list of neighbors.
         *
         * Missing checks:
         *   optlen = 1 (8 octets)
         *   Should only update Neighbor Table if [O]verride bit is set in flags
         */

        adv = ICMPv6ADVERTISE;
        if (net_ipv6addr_cmp(icmp->destipaddr, dev->d_ipv6addr))
          {
            /* This message is required to support the Target link-layer
             * address option.
             */

            if (adv->opttype == ICMPv6_OPT_TGTLLADDR)
              {
                /* Save the sender's address mapping in our Neighbor Table. */

                neighbor_add(icmp->srcipaddr,
                             (FAR struct neighbor_addr_s *)adv->tgtlladdr);

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
                /* Then notify any logic waiting for the Neighbor Advertisement */

                icmpv6_notify(icmp->srcipaddr);
#endif

                /* We consumed the packet but we don't send anything in
                 * response.
                 */

                goto icmpv_send_nothing;
              }
          }

        goto icmpv6_drop_packet;
      }
      break;

#ifdef CONFIG_NET_ICMPv6_ROUTER
    /* Check if we received a Router Solicitation */

    case ICMPV6_ROUTER_SOLICIT:
      {
        /* Just give a knee-jerk Router Advertisement in respond with no
         * further examination of the Router Solicitation.
         */

        icmpv6_radvertise(dev);

        /* All statistics have been updated.  Nothing to do but exit. */

        return;
      }
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
    /* Check if we received a Router Advertisement */

    case ICMPV6_ROUTER_ADVERTISE:
      {
        FAR struct icmpv6_router_advertise_s *adv;
        uint16_t pktlen;
        uint16_t optlen;
        int ndx;

        /* Get the length of the option data */

        pktlen = (uint16_t)icmp->len[0] << 8 | icmp->len[1];
        if (pktlen <= ICMPv6_RADV_MINLEN)
          {
            /* Too small to contain any options */

            goto icmpv6_drop_packet;
          }

        optlen = ICMPv6_RADV_OPTLEN(pktlen);

        /* We need to have a valid router advertisement with a Prefix and
         * with the "A" bit set in the flags.
         */

        adv = ICMPv6RADVERTISE;
        for (ndx = 0; ndx + sizeof(struct icmpv6_prefixinfo_s) <= optlen; )
          {
            FAR struct icmpv6_prefixinfo_s *opt =
              (FAR struct icmpv6_prefixinfo_s *)&adv->options[ndx];

            /* Is this the sought for prefix? Is it the correct size? Is
             * the "A" flag set?
             */

            if (opt->opttype &&
                opt->optlen == 4 &&
               (opt->flags & ICMPv6_PRFX_FLAG_A) != 0)
              {
                /* Yes.. Notify any waiting threads */

                icmpv6_rnotify(dev, icmp->srcipaddr, opt->prefix, opt->preflen);
                goto icmpv_send_nothing;
              }

            /* Skip to the next option (units of octets) */

            ndx += (opt->optlen << 3);
          }

        goto icmpv6_drop_packet;
      }
      break;
#endif

    /* Handle the ICMPv6 Echo Request */

    case ICMPv6_ECHO_REQUEST:
      {
        /* ICMPv6 echo (i.e., ping) processing. This is simple, we only
         * change the ICMPv6 type from ECHO to ECHO_REPLY and update the
         * ICMPv6 checksum before we return the packet.
         */

        icmp->type = ICMPv6_ECHO_REPLY;

        net_ipv6addr_copy(icmp->destipaddr, icmp->srcipaddr);
        net_ipv6addr_copy(icmp->srcipaddr, dev->d_ipv6addr);

        icmp->chksum = 0;
        icmp->chksum = ~icmpv6_chksum(dev);
      }
      break;

#ifdef CONFIG_NET_ICMPv6_PING
    /* If an ICMPv6 echo reply is received then there should also be
     * a thread waiting to received the echo response.
     */

    case ICMPv6_ECHO_REPLY:
      {
        uint16_t flags = ICMPv6_ECHOREPLY;

        /* Dispatch the ECHO reply to the waiting thread */

        flags = devif_conn_event(dev, icmp, flags, dev->d_conncb);

        /* If the ECHO reply was not handled, then drop the packet */

        if (flags == ICMPv6_ECHOREPLY)
          {
            /* The ECHO reply was not handled */

            goto icmpv6_drop_packet;
          }
      }
      break;
#endif

    default:
      {
        nlldbg("Unknown ICMPv6 type: %d\n", icmp->type);
        goto icmpv6_type_error;
      }
    }

  nllvdbg("Outgoing ICMPv6 packet length: %d (%d)\n",
          dev->d_len, (icmp->len[0] << 8) | icmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
  return;

icmpv6_type_error:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.typeerr++;
#endif

icmpv6_drop_packet:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.drop++;
#endif

icmpv_send_nothing:
  dev->d_len = 0;
}

#endif /* CONFIG_NET_ICMPv6 */
